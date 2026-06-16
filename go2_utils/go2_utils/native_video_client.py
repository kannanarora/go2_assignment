#!/usr/bin/env python3

"""Minimal Unitree Go2 video RPC client.

This is a small, project-local extraction of the Unitree SDK2 Python video
client path. It only includes the pieces needed to call videohub/GetImageSample.
"""

import time
from dataclasses import dataclass
from enum import Enum
from threading import Condition, Lock

from cyclonedds.core import DDSException, Listener
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.idl import IdlStruct
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from cyclonedds.internal import InvalidSample, dds_c_t
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from cyclonedds.util import duration


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestIdentity_(IdlStruct, typename="unitree_api.msg.dds_.RequestIdentity_"):
    id: types.int64
    api_id: types.int64


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestLease_(IdlStruct, typename="unitree_api.msg.dds_.RequestLease_"):
    id: types.int64


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestPolicy_(IdlStruct, typename="unitree_api.msg.dds_.RequestPolicy_"):
    priority: types.int32
    noreply: bool


@dataclass
@annotate.final
@annotate.autoid("sequential")
class RequestHeader_(IdlStruct, typename="unitree_api.msg.dds_.RequestHeader_"):
    identity: RequestIdentity_
    lease: RequestLease_
    policy: RequestPolicy_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Request_(IdlStruct, typename="unitree_api.msg.dds_.Request_"):
    header: RequestHeader_
    parameter: str
    binary: types.sequence[types.uint8]


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ResponseStatus_(IdlStruct, typename="unitree_api.msg.dds_.ResponseStatus_"):
    code: types.int32


@dataclass
@annotate.final
@annotate.autoid("sequential")
class ResponseHeader_(IdlStruct, typename="unitree_api.msg.dds_.ResponseHeader_"):
    identity: RequestIdentity_
    status: ResponseStatus_


@dataclass
@annotate.final
@annotate.autoid("sequential")
class Response_(IdlStruct, typename="unitree_api.msg.dds_.Response_"):
    header: ResponseHeader_
    data: str
    binary: types.sequence[types.uint8]


CHANNEL_CONFIG_HAS_INTERFACE = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$__IF_NAME__$" priority="default" multicast="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""


CHANNEL_CONFIG_AUTO = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS>
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""


class FutureResult:
    FUTURE_SUCC = 0
    FUTUTE_ERR_TIMEOUT = 1
    FUTURE_ERR_FAILED = 2
    FUTURE_ERR_UNKNOWN = 3

    def __init__(self, code: int, msg: str, value=None):
        self.code = code
        self.msg = msg
        self.value = value


class FutureState(Enum):
    DEFER = 0
    READY = 1
    FAILED = 2


class Future:
    def __init__(self):
        self._state = FutureState.DEFER
        self._value = None
        self._msg = None
        self._condition = Condition()

    def GetResult(self, timeout: float = None):
        with self._condition:
            return self._wait_result(timeout)

    def Ready(self, value):
        with self._condition:
            if self._state != FutureState.DEFER:
                return False
            self._value = value
            self._state = FutureState.READY
            self._condition.notify()
            return True

    def _wait_result(self, timeout: float = None):
        if self._state == FutureState.DEFER:
            if not self._condition.wait(timeout):
                return FutureResult(FutureResult.FUTUTE_ERR_TIMEOUT, "future timeout")

        if self._state == FutureState.READY:
            return FutureResult(FutureResult.FUTURE_SUCC, "success", self._value)
        if self._state == FutureState.FAILED:
            return FutureResult(FutureResult.FUTURE_ERR_FAILED, self._msg)
        return FutureResult(FutureResult.FUTURE_ERR_UNKNOWN, "future state error")


class RequestFuture(Future):
    def __init__(self):
        self._request_id = None
        super().__init__()

    def SetRequestId(self, request_id: int):
        self._request_id = request_id


class RequestFutureQueue:
    def __init__(self):
        self._data = {}
        self._lock = Lock()

    def Set(self, request_id: int, future: RequestFuture):
        with self._lock:
            self._data[request_id] = future

    def Get(self, request_id: int):
        with self._lock:
            return self._data.pop(request_id, None)

    def Remove(self, request_id: int):
        with self._lock:
            self._data.pop(request_id, None)


class ChannelType(Enum):
    SEND = 0
    RECV = 1


def get_client_channel_name(service_name: str, channel_type: ChannelType):
    suffix = "request" if channel_type == ChannelType.SEND else "response"
    return "rt/api/%s/%s" % (service_name, suffix)


class Channel:
    def __init__(self, participant: DomainParticipant, name: str, sample_type):
        self._participant = participant
        self._topic = Topic(participant, name, sample_type)
        self._reader = None
        self._writer = None
        self._publication_matched_count = 0

    def SetWriter(self):
        self._writer = DataWriter(
            self._participant,
            self._topic,
            listener=Listener(on_publication_matched=self._on_publication_matched),
        )
        time.sleep(0.2)

    def SetReader(self, handler):
        self._reader = DataReader(
            self._participant,
            self._topic,
            listener=Listener(on_data_available=lambda reader: self._on_data(reader, handler)),
        )

    def Write(self, sample, timeout: float = None):
        wait_s = 0.0 if timeout is None else timeout
        while wait_s > 0.0 and self._publication_matched_count == 0:
            time.sleep(0.1)
            wait_s -= 0.1

        if timeout is not None and wait_s <= 0.0 and self._publication_matched_count == 0:
            return False

        try:
            self._writer.write(sample)
        except DDSException as exc:
            print("[VideoClient] DDS write error:", exc)
            return False
        return True

    def _on_publication_matched(self, writer, status: dds_c_t.publication_matched_status):
        self._publication_matched_count = status.current_count

    def _on_data(self, reader: DataReader, handler):
        try:
            samples = reader.take(1)
        except (DDSException, TimeoutError):
            return

        if not samples or isinstance(samples[0], InvalidSample):
            return

        handler(samples[0])


class ChannelFactory:
    _domain = None
    _participant = None
    _initialized = False

    @classmethod
    def Init(cls, domain_id: int = 0, network_interface: str = None):
        if cls._initialized:
            return

        config = CHANNEL_CONFIG_AUTO
        if network_interface:
            config = CHANNEL_CONFIG_HAS_INTERFACE.replace(
                "$__IF_NAME__$",
                network_interface,
            )

        cls._domain = Domain(domain_id, config)
        cls._participant = DomainParticipant(domain_id)
        cls._initialized = True

    @classmethod
    def CreateSendChannel(cls, name: str, sample_type):
        channel = Channel(cls._participant, name, sample_type)
        channel.SetWriter()
        return channel

    @classmethod
    def CreateRecvChannel(cls, name: str, sample_type, handler):
        channel = Channel(cls._participant, name, sample_type)
        channel.SetReader(handler)
        return channel


def ChannelFactoryInitialize(domain_id: int = 0, network_interface: str = None):
    ChannelFactory.Init(domain_id, network_interface)


RPC_INTERNAL_API_ID_MAX = 100
RPC_ERR_UNKNOWN = 3001
RPC_ERR_CLIENT_SEND = 3102
RPC_ERR_CLIENT_API_NOT_REG = 3103
RPC_ERR_CLIENT_API_TIMEOUT = 3104
RPC_ERR_CLIENT_API_NOT_MATCH = 3105

VIDEO_SERVICE_NAME = "videohub"
VIDEO_API_VERSION = "1.0.0.1"
VIDEO_API_ID_GETIMAGESAMPLE = 1001


class ClientStub:
    def __init__(self, service_name: str):
        self._service_name = service_name
        self._future_queue = RequestFutureQueue()
        self._send_channel = None
        self._recv_channel = None

    def Init(self):
        self._send_channel = ChannelFactory.CreateSendChannel(
            get_client_channel_name(self._service_name, ChannelType.SEND),
            Request_,
        )
        self._recv_channel = ChannelFactory.CreateRecvChannel(
            get_client_channel_name(self._service_name, ChannelType.RECV),
            Response_,
            self._response_handler,
        )
        time.sleep(0.5)

    def SendRequest(self, request: Request_, timeout: float):
        request_id = request.header.identity.id
        future = RequestFuture()
        future.SetRequestId(request_id)
        self._future_queue.Set(request_id, future)

        if self._send_channel.Write(request, timeout):
            return future

        self._future_queue.Remove(request_id)
        return None

    def RemoveFuture(self, request_id: int):
        self._future_queue.Remove(request_id)

    def _response_handler(self, response: Response_):
        future = self._future_queue.Get(response.header.identity.id)
        if future is not None:
            future.Ready(response)


class ClientBase:
    def __init__(self, service_name: str):
        self._timeout = 1.0
        self._stub = ClientStub(service_name)
        self._stub.Init()

    def SetTimeout(self, timeout: float):
        self._timeout = timeout

    def _CallBinaryBase(self, api_id: int, parameter: list, priority: int, lease_id: int):
        request = Request_(self._make_header(api_id, lease_id, priority, False), "", parameter)
        future = self._stub.SendRequest(request, self._timeout)
        if future is None:
            return RPC_ERR_CLIENT_SEND, None

        result = future.GetResult(self._timeout)
        if result.code != FutureResult.FUTURE_SUCC:
            self._stub.RemoveFuture(request.header.identity.id)
            code = RPC_ERR_CLIENT_API_TIMEOUT
            if result.code != FutureResult.FUTUTE_ERR_TIMEOUT:
                code = RPC_ERR_UNKNOWN
            return code, None

        response = result.value
        if response.header.identity.api_id != api_id:
            return RPC_ERR_CLIENT_API_NOT_MATCH, None
        return response.header.status.code, response.binary

    def _make_header(self, api_id: int, lease_id: int, priority: int, no_reply: bool):
        identity = RequestIdentity_(time.monotonic_ns(), api_id)
        lease = RequestLease_(lease_id)
        policy = RequestPolicy_(priority, no_reply)
        return RequestHeader_(identity, lease, policy)


class Client(ClientBase):
    def __init__(self, service_name: str):
        super().__init__(service_name)
        self._api_mapping = {}
        self._api_version = None

    def _SetApiVerson(self, api_version: str):
        self._api_version = api_version

    def _RegistApi(self, api_id: int, priority: int):
        self._api_mapping[api_id] = priority

    def _CallBinary(self, api_id: int, parameter: list):
        if api_id > RPC_INTERNAL_API_ID_MAX and api_id not in self._api_mapping:
            return RPC_ERR_CLIENT_API_NOT_REG, None

        priority = self._api_mapping.get(api_id, 0)
        return self._CallBinaryBase(api_id, parameter, priority, 0)


class VideoClient(Client):
    def __init__(self):
        super().__init__(VIDEO_SERVICE_NAME)

    def Init(self):
        self._SetApiVerson(VIDEO_API_VERSION)
        self._RegistApi(VIDEO_API_ID_GETIMAGESAMPLE, 0)

    def GetImageSample(self):
        return self._CallBinary(VIDEO_API_ID_GETIMAGESAMPLE, [])
