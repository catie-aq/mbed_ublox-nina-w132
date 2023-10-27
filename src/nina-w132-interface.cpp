/* NINAW132 implementation of NetworkInterfaceAPI
 * Copyright (c) 2015 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// #if DEVICE_SERIAL && DEVICE_INTERRUPTIN && defined(MBED_CONF_EVENTS_PRESENT) &&
// defined(MBED_CONF_NSAPI_PRESENT) && defined(MBED_CONF_RTOS_API_PRESENT)

#include <stdint.h>
#include <string.h>

#include "nina-w132-interface.h"

using namespace std::chrono;

#define NINAW132_WIFI_IF_NAME "ni0"

using namespace mbed;
using namespace rtos;

// Set 1 to force debug information
#define ninaw132_interface_debug 1

// NINAW132Interface implementation
NINAW132Interface::NINAW132Interface(bool debug):
        _ninaw132(MBED_CONF_NINA_W132_TX,
                MBED_CONF_NINA_W132_RX,
                MBED_CONF_NINA_W132_RST,
                MBED_CONF_NINA_W132_DEBUG),
        _ap_sec(NSAPI_SECURITY_UNKNOWN),
        _if_blocking(true),
#if MBED_CONF_RTOS_PRESENT
        _if_connected(_cmutex),
        _if_socket_opened(_smutex),
#endif
        _socket_stat_cb(0),
        _initialized(false),
        _connect_retval(NSAPI_ERROR_OK),
        _disconnect_retval(NSAPI_ERROR_OK),
        _conn_stat(NSAPI_STATUS_DISCONNECTED),
        _conn_stat_cb(),
        _global_event_queue(
                mbed_event_queue()), // Needs to be set before attaching event() to SIGIO
        _oob_event_id(0),
        _connect_event_id(0),
        _disconnect_event_id(0),
        _software_conn_stat(IFACE_STATUS_DISCONNECTED),
        _dhcp(true)
{
    _ninaw132_interface_debug = ninaw132_interface_debug || debug;
    memset(_cbs, 0, sizeof(_cbs));
    memset(ap_ssid, 0, sizeof(ap_ssid));
    memset(ap_pass, 0, sizeof(ap_pass));

    _ninaw132.sigio(this, &NINAW132Interface::event);
    _ninaw132.set_timeout();
    _ninaw132.attach(this, &NINAW132Interface::refresh_conn_state_cb);
    // _ninaw132.attach_socket_recv(this, &NINAW132Interface::refresh_socket_data_state_cb);
    _ninaw132.attach_socket_open(this, &NINAW132Interface::refresh_socket_open_state_cb);

    for (int i = 0; i < NINAW132_SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].sport = 0;
    }

    _global_event_queue->call(callback(this, &NINAW132Interface::_init));
}

NINAW132Interface::~NINAW132Interface()
{
    if (_oob_event_id) {
        _global_event_queue->cancel(_oob_event_id);
    }

    _cmutex.lock();
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
    }
    _cmutex.unlock();
}

int NINAW132Interface::connect(
        const char *ssid, const char *pass, nsapi_security_t security, uint8_t channel)
{
    if (channel != 0) {
        return NSAPI_ERROR_UNSUPPORTED;
    }

    int err = set_credentials(ssid, pass, security);
    if (err) {
        return err;
    }

    return connect();
}

void NINAW132Interface::_connect_async()
{
    if (_dhcp && !_ninaw132.dhcp(true)) {
        _connect_retval = NSAPI_ERROR_DHCP_FAILURE;
        _ninaw132.uart_enable_input(false);
        _software_conn_stat = IFACE_STATUS_DISCONNECTED;
        //_conn_stat_cb will be called from refresh_conn_state_cb
        return;
    }
    _cmutex.lock();
    if (!_connect_event_id) {
        debug_if(_ninaw132_interface_debug, "_connect_async(): Cancelled.");
        _cmutex.unlock();
        return;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {
        _ninaw132.authentification_type(_ap_sec);
    }

    _connect_retval = _ninaw132.connect(ap_ssid, ap_pass, _ap_sec);
    auto timepassed = _conn_timer.elapsed_time();
    if (_connect_retval == NSAPI_ERROR_OK || _connect_retval == NSAPI_ERROR_AUTH_FAILURE
            || _connect_retval == NSAPI_ERROR_NO_SSID
            || ((_if_blocking == true) && (timepassed >= NINAW132_INTERFACE_CONNECT_TIMEOUT))) {
        _connect_event_id = 0;
        _conn_timer.stop();
        if (timepassed >= NINAW132_INTERFACE_CONNECT_TIMEOUT && _connect_retval != NSAPI_ERROR_OK) {
            _connect_retval = NSAPI_ERROR_CONNECTION_TIMEOUT;
        }
        if (_connect_retval != NSAPI_ERROR_OK) {
            _ninaw132.uart_enable_input(false);
            _software_conn_stat = IFACE_STATUS_DISCONNECTED;
        }
#if MBED_CONF_RTOS_PRESENT
        _if_connected.notify_all();
#endif
    } else {
        // Postpone to give other stuff time to run
        _connect_event_id = _global_event_queue->call_in(NINAW132_INTERFACE_CONNECT_INTERVAL,
                callback(this, &NINAW132Interface::_connect_async));
        if (!_connect_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                    "NINAW132Interface::_connect_async(): unable to add event to queue. Increase "
                    "\"events.shared-eventsize\"\n");
        }
    }
    _cmutex.unlock();

    if (_connect_event_id == 0) {
        if (_conn_stat_cb) {
            _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
        }
        if (_conn_stat == NSAPI_STATUS_GLOBAL_UP || _conn_stat == NSAPI_STATUS_LOCAL_UP) {
            _software_conn_stat = IFACE_STATUS_CONNECTED;
        }
    }
}

int NINAW132Interface::connect()
{
    if (_software_conn_stat == IFACE_STATUS_CONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (_software_conn_stat == IFACE_STATUS_CONNECTED) {
        return NSAPI_ERROR_IS_CONNECTED;
    }

    if (strlen(ap_ssid) == 0) {
        return NSAPI_ERROR_NO_SSID;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {
        if (strlen(ap_pass) < NINAW132_PASSPHRASE_MIN_LENGTH) {
            return NSAPI_ERROR_PARAMETER;
        }
    }
    if (!_if_blocking) {
        bool ret = _cmutex.trylock();
        if (ret == false) {
            return NSAPI_ERROR_BUSY;
        }
    } else {
        _cmutex.lock();
    }
    _software_conn_stat = IFACE_STATUS_CONNECTING;
    _ninaw132.uart_enable_input(true);
    _connect_retval = NSAPI_ERROR_NO_CONNECTION;
    MBED_ASSERT(!_connect_event_id);
    _conn_timer.stop();
    _conn_timer.reset();
    _conn_timer.start();
    _connect_event_id
            = _global_event_queue->call(callback(this, &NINAW132Interface::_connect_async));

    if (!_connect_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                "connect(): unable to add event to queue. Increase \"events.shared-eventsize\"\n");
    }

#if MBED_CONF_RTOS_PRESENT
    while (_if_blocking && (_conn_status_to_error() != NSAPI_ERROR_IS_CONNECTED)
            && (_connect_retval == NSAPI_ERROR_NO_CONNECTION)) {
        _if_connected.wait();
    }
#endif

    _cmutex.unlock();

    if (!_if_blocking) {
        return NSAPI_ERROR_OK;
    } else {
        return _connect_retval;
    }
}

int NINAW132Interface::set_credentials(
        const char *ssid, const char *pass, nsapi_security_t security)
{
    nsapi_error_t status = _conn_status_to_error();
    if (_software_conn_stat == IFACE_STATUS_CONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (status != NSAPI_ERROR_NO_CONNECTION) {
        return status;
    }

    _ap_sec = security;

    if (!ssid) {
        return NSAPI_ERROR_PARAMETER;
    }

    int ssid_length = strlen(ssid);

    if (ssid_length > 0 && ssid_length <= NINAW132_SSID_MAX_LENGTH) {
        memset(ap_ssid, 0, sizeof(ap_ssid));
        strncpy(ap_ssid, ssid, NINAW132_SSID_MAX_LENGTH);
    } else {
        return NSAPI_ERROR_PARAMETER;
    }

    if (_ap_sec != NSAPI_SECURITY_NONE) {

        if (!pass) {
            return NSAPI_ERROR_PARAMETER;
        }

        int pass_length = strlen(pass);
        if (pass_length >= NINAW132_PASSPHRASE_MIN_LENGTH
                && pass_length <= NINAW132_PASSPHRASE_MAX_LENGTH) {
            memset(ap_pass, 0, sizeof(ap_pass));
            strncpy(ap_pass, pass, NINAW132_PASSPHRASE_MAX_LENGTH);
        } else {
            return NSAPI_ERROR_PARAMETER;
        }
    } else {
        memset(ap_pass, 0, sizeof(ap_pass));
    }

    return NSAPI_ERROR_OK;
}

int NINAW132Interface::set_channel(uint8_t channel)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

nsapi_error_t NINAW132Interface::set_network(
        const SocketAddress &ip_address, const SocketAddress &netmask, const SocketAddress &gateway)
{
    nsapi_error_t init_result = _init();
    if (NSAPI_ERROR_OK != init_result) {
        return init_result;
    }

    // netmask and gateway
    if (_ninaw132.set_ip_addr(
                ip_address.get_ip_address(), gateway.get_ip_address(), netmask.get_ip_address())) {
        _dhcp = false;
        return NSAPI_ERROR_OK;
    } else {
        return NSAPI_ERROR_DEVICE_ERROR;
    }
}

nsapi_error_t NINAW132Interface::set_dhcp(bool dhcp)
{
    nsapi_error_t init_result = _init();
    if (NSAPI_ERROR_OK != init_result) {
        return init_result;
    }

    _dhcp = dhcp;
    if (_ninaw132.dhcp(dhcp)) {
        return NSAPI_ERROR_OK;
    } else {
        return NSAPI_ERROR_DEVICE_ERROR;
    }
}

void NINAW132Interface::_disconnect_async()
{
    _cmutex.lock();
    _disconnect_retval = _ninaw132.disconnect() ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
    auto timepassed = _conn_timer.elapsed_time();

    if (_disconnect_retval == NSAPI_ERROR_OK
            || ((_if_blocking == true) && (timepassed >= NINAW132_INTERFACE_CONNECT_TIMEOUT))) {

        if (timepassed >= NINAW132_INTERFACE_CONNECT_TIMEOUT && _connect_retval != NSAPI_ERROR_OK) {
            _disconnect_retval = NSAPI_ERROR_CONNECTION_TIMEOUT;
        } else {
            if (_conn_stat != NSAPI_STATUS_DISCONNECTED) {
                _conn_stat = NSAPI_STATUS_DISCONNECTED;
            }
            // In case the status update arrives later inform upper layers manually
            _disconnect_event_id = 0;
            _conn_timer.stop();
            _connect_retval = NSAPI_ERROR_NO_CONNECTION;
        }

        _software_conn_stat = IFACE_STATUS_DISCONNECTED;
#if MBED_CONF_RTOS_PRESENT
        _if_connected.notify_all();
#endif

    } else {
        // Postpone to give other stuff time to run
        _disconnect_event_id = _global_event_queue->call_in(NINAW132_INTERFACE_CONNECT_INTERVAL,
                callback(this, &NINAW132Interface::_disconnect_async));
        if (!_disconnect_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                    "NINAW132Interface::_disconnect_async(): unable to add event to queue. "
                    "Increase \"events.shared-eventsize\"\n");
        }
    }
    _cmutex.unlock();

    _ninaw132.uart_enable_input(false);
    if (_disconnect_event_id == 0) {
        if (_conn_stat_cb) {
            _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
        }
    }
}

int NINAW132Interface::disconnect()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTING) {
        return NSAPI_ERROR_BUSY;
    }
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }
    if (!_if_blocking) {
        bool ret = _cmutex.trylock();
        if (ret == false) {
            return NSAPI_ERROR_BUSY;
        }
    } else {
        _cmutex.lock();
    }
    if (_connect_event_id) {
        _global_event_queue->cancel(_connect_event_id);
        _connect_event_id = 0; // cancel asynchronous connection attempt if one is ongoing
    }
    _software_conn_stat = IFACE_STATUS_DISCONNECTING;

    _disconnect_retval = NSAPI_ERROR_IS_CONNECTED;
    _disconnect_event_id = 0;

    _initialized = false;
    _conn_timer.stop();
    _conn_timer.reset();
    _conn_timer.start();

    _disconnect_event_id
            = _global_event_queue->call(callback(this, &NINAW132Interface::_disconnect_async));

    if (!_disconnect_event_id) {
        MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                "disconnect(): unable to add event to queue. Increase "
                "\"events.shared-eventsize\"\n");
    }

#if MBED_CONF_RTOS_PRESENT
    while (_if_blocking && (_conn_status_to_error() != NSAPI_ERROR_NO_CONNECTION)
            && (_disconnect_retval != NSAPI_ERROR_OK)) {
        _if_connected.wait();
    }
#endif

    _cmutex.unlock();
    if (!_if_blocking) {
        return NSAPI_ERROR_OK;
    } else {
        return _disconnect_retval;
    }
}

nsapi_error_t NINAW132Interface::get_ip_address(SocketAddress *address)
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(true);
    }

    const char *ip_buff = _ninaw132.getIPAddress();
    if (!ip_buff || strcmp(ip_buff, "0.0.0.0") == 0) {
        ip_buff = NULL;
    }
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(false);
    }
    if (ip_buff) {
        address->set_ip_address(ip_buff);
        return NSAPI_ERROR_OK;
    }
    return NSAPI_ERROR_NO_ADDRESS;
}

const char *NINAW132Interface::get_mac_address()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(true);
    }
    const char *ret = _ninaw132.getMACAddress();

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(false);
    }
    return ret;
}

nsapi_error_t NINAW132Interface::get_gateway(SocketAddress *address)
{
    if (address == nullptr) {
        return NSAPI_ERROR_PARAMETER;
    }
    if (_conn_stat == NSAPI_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    if (!address->set_ip_address(_ninaw132.getGateway())) {
        return NSAPI_ERROR_NO_ADDRESS;
    }

    return NSAPI_ERROR_OK;
}

const char *NINAW132Interface::get_gateway()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _ninaw132.getGateway() : NULL;
}

nsapi_error_t NINAW132Interface::get_netmask(SocketAddress *address)
{
    if (address == nullptr) {
        return NSAPI_ERROR_PARAMETER;
    }
    if (_conn_stat == NSAPI_STATUS_DISCONNECTED) {
        return NSAPI_ERROR_NO_CONNECTION;
    }

    if (!address->set_ip_address(_ninaw132.getNetmask())) {
        return NSAPI_ERROR_NO_ADDRESS;
    }

    return NSAPI_ERROR_OK;
}

const char *NINAW132Interface::get_netmask()
{
    return _conn_stat != NSAPI_STATUS_DISCONNECTED ? _ninaw132.getNetmask() : NULL;
}

char *NINAW132Interface::get_interface_name(char *interface_name)
{
    memcpy(interface_name, NINAW132_WIFI_IF_NAME, sizeof(NINAW132_WIFI_IF_NAME));
    return interface_name;
}

int8_t NINAW132Interface::get_rssi()
{
    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(true);
    }

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(false);
        return 0;
    }

    return _ninaw132.getRSSI();
}

int NINAW132Interface::scan(WiFiAccessPoint *res, unsigned count)
{

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(true);
    }

    int ret = _ninaw132.scan(res, count);

    if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
        _ninaw132.uart_enable_input(false);
    }

    return ret;
}

bool NINAW132Interface::_get_firmware_version()
{
    NINAW132::fw_at_version at_v = _ninaw132.get_firmware_version();
    debug_if(_ninaw132_interface_debug,
            "NINAW132: Firmware v%d.%d.%d\r\n",
            at_v.major,
            at_v.minor,
            at_v.patch);

    return true;
}

nsapi_error_t NINAW132Interface::_init(void)
{
    if (!_initialized) {
        // check the presence of module
        if (!_ninaw132.at_available()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_ninaw132.echo_off()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        if (!_get_firmware_version()) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        // TODO: set wifi default mode

        _initialized = true;
    }
    return NSAPI_ERROR_OK;
}

nsapi_error_t NINAW132Interface::_reset()
{
    _ninaw132.hardware_reset();
    _ninaw132.uart_enable_input(true);

    return _ninaw132.at_available() ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
}

int NINAW132Interface::socket_open(void **handle, nsapi_protocol_t proto)
{
    struct nina_w132_socket *socket = new struct nina_w132_socket;
    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = -1; // set to the socket connect
    socket->proto = proto;
    socket->connected = false;
    socket->bound = false;
    socket->keepalive = 0;
    *handle = socket;
    return 0;
}

int NINAW132Interface::_socket_open(void *handle)
{
    // Look for an unused socket
    int id = -1;

    for (int i = 0; i < NINAW132_SOCKET_COUNT; i++) {
        if (!_sock_i[i].open) {
            id = i;
            _sock_i[i].open = true;
            break;
        }
    }

    if (id == -1) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    socket->id = id;
    handle = socket;

    debug_if(_ninaw132_interface_debug, "[Network Interface] [socket open] id: %d\n", id);

    return 0;
}

int NINAW132Interface::socket_close(void *handle)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    int err = 0;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->connected && !_ninaw132.close(socket->id)) {
        err = NSAPI_ERROR_DEVICE_ERROR;
    }

    if (socket->bound && !_ninaw132.close(socket->id)) {
        err = NSAPI_ERROR_DEVICE_ERROR;
    }

    _cbs[socket->id].callback = NULL;
    _cbs[socket->id].data = NULL;
    core_util_atomic_store_u8(&_cbs[socket->id].deferred, false);

    socket->connected = false;
    socket->bound = false;
    _sock_i[socket->id].open = false;
    _sock_i[socket->id].sport = 0;

    debug_if(_ninaw132_interface_debug, "[Network Interface] [socket close] id: %d\n", socket->id);

    delete socket;
    
    return err;
}

int NINAW132Interface::socket_bind(void *handle, const SocketAddress &address)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (socket->proto == NSAPI_UDP) {
        if (address.get_addr().version != NSAPI_UNSPEC) {
            return NSAPI_ERROR_UNSUPPORTED;
        }

        for (int id = 0; id < NINAW132_SOCKET_COUNT; id++) {
            if (_sock_i[id].sport == address.get_port()
                    && id != socket->id) { // Port already reserved by another socket
                return NSAPI_ERROR_PARAMETER;
            } else if (id == socket->id && (socket->connected || socket->bound)) {
                return NSAPI_ERROR_PARAMETER;
            }
        }
        _sock_i[socket->id].sport = address.get_port();
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

int NINAW132Interface::socket_listen(void *handle, int backlog)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int NINAW132Interface::socket_connect(void *handle, const SocketAddress &addr)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    nsapi_error_t ret;
    cv_status status;

    // manage socket id for the NINAW132 module
    _socket_open(handle);

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    debug_if(_ninaw132_interface_debug, "[Network Interface] [socket connect] socket id: %d\n", socket->id);

    if (socket->proto == NSAPI_UDP) {
        ret = _ninaw132.open_udp(socket->id, addr.get_ip_address(), addr.get_port());
    } else {
        ret = _ninaw132.open_tcp(
                socket->id, addr.get_ip_address(), addr.get_port(), socket->keepalive);
    }

    // wait socket connection
    _smutex.lock();
#if MBED_CONF_RTOS_PRESENT
    status = _if_socket_opened.wait_for(NINA_W132_OPEN_TIMEOUT);
#endif

    _smutex.unlock();

    socket->connected = (ret == NSAPI_ERROR_OK) ? true : false;

    return ret;
}

int NINAW132Interface::socket_accept(void *server, void **socket, SocketAddress *addr)
{
    return NSAPI_ERROR_UNSUPPORTED;
}

int NINAW132Interface::socket_send(void *handle, const void *data, unsigned size)
{
    nsapi_size_or_error_t status;
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    uint8_t expect_false = false;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!_sock_i[socket->id].open) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    if (!size) {
        // Firmware limitation
        return socket->proto == NSAPI_TCP ? 0 : NSAPI_ERROR_UNSUPPORTED;
    }

    if (socket->proto == NSAPI_TCP) {
        status = _ninaw132.send_tcp(socket->id, data, size);
    } else {
        status = _ninaw132.send_udp(socket->id, data, size);
    }

    return status;
}

int NINAW132Interface::socket_recv(void *handle, void *data, unsigned size)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    int32_t recv = 0;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (!_sock_i[socket->id].open) {
        return NSAPI_ERROR_CONNECTION_LOST;
    }

    _smutex.lock();

    if (socket->proto == NSAPI_TCP) {
        recv = _ninaw132.recv_tcp(socket->id, data, size);
    } else {
        recv = _ninaw132.recv_udp(socket->id, data, size);
    }

    _smutex.unlock();
    
    if (recv < 0 ) {
        return NSAPI_ERROR_WOULD_BLOCK;
    }

    return recv;
}

int NINAW132Interface::socket_sendto(
        void *handle, const SocketAddress &addr, const void *data, unsigned size)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if ((strcmp(addr.get_ip_address(), "0.0.0.0") == 0) || !addr.get_port()) {
        return NSAPI_ERROR_DNS_FAILURE;
    }

    if (socket->connected && socket->addr != addr) {
        if (!_ninaw132.close(socket->id)) {
            return NSAPI_ERROR_DEVICE_ERROR;
        }
        socket->connected = false;
    }

    if (!socket->connected && !socket->bound) {
        int err = socket_connect(socket, addr);
        if (err < 0) {
            return err;
        }
        socket->addr = addr;
    }

    if (socket->bound) {
        socket->addr = addr;
    }

    return socket_send(socket, data, size);
}

int NINAW132Interface::socket_recvfrom(void *handle, SocketAddress *addr, void *data, unsigned size)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;

    if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    int ret = socket_recv(socket, data, size);
    if (ret >= 0 && addr) {
        *addr = socket->addr;
    }

    return ret;
}

void NINAW132Interface::socket_attach(void *handle, void (*callback)(void *), void *data)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;
    _cbs[socket->id].callback = callback;
    _cbs[socket->id].data = data;
}

nsapi_error_t NINAW132Interface::setsockopt(
        nsapi_socket_t handle, int level, int optname, const void *optval, unsigned optlen)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;

    if (!optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (socket->connected) { // NINAW132 limitation, keepalive needs to be given before
                                         // connecting
                    return NSAPI_ERROR_UNSUPPORTED;
                }

                if (optlen == sizeof(int)) {
                    int secs = *(int *)optval;
                    if (secs >= 0 && secs <= 7200) {
                        socket->keepalive = secs;
                        return NSAPI_ERROR_OK;
                    }
                }
                return NSAPI_ERROR_PARAMETER;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

nsapi_error_t NINAW132Interface::getsockopt(
        nsapi_socket_t handle, int level, int optname, void *optval, unsigned *optlen)
{
    struct nina_w132_socket *socket = (struct nina_w132_socket *)handle;

    if (!optval || !optlen) {
        return NSAPI_ERROR_PARAMETER;
    } else if (!socket) {
        return NSAPI_ERROR_NO_SOCKET;
    }

    if (level == NSAPI_SOCKET && socket->proto == NSAPI_TCP) {
        switch (optname) {
            case NSAPI_KEEPALIVE: {
                if (*optlen > sizeof(int)) {
                    *optlen = sizeof(int);
                }
                memcpy(optval, &(socket->keepalive), *optlen);
                return NSAPI_ERROR_OK;
            }
        }
    }

    return NSAPI_ERROR_UNSUPPORTED;
}

void NINAW132Interface::event()
{
    if (!_oob_event_id) {
        // Throttles event creation by using arbitrary small delay
        _oob_event_id = _global_event_queue->call_in(
                50ms, callback(this, &NINAW132Interface::proc_oob_evnt));
        if (!_oob_event_id) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_ENOMEM),
                    "NINAW132Interface::event(): unable to add event to queue. Increase "
                    "\"events.shared-eventsize\"\n");
        }
    }

    for (int i = 0; i < NINAW132_SOCKET_COUNT; i++) {
        if (_cbs[i].callback) {
            _cbs[i].callback(_cbs[i].data);
        }
    }
}

void NINAW132Interface::event_deferred()
{
    for (int i = 0; i < NINAW132_SOCKET_COUNT; i++) {
        uint8_t expect_true = true;
        if (core_util_atomic_cas_u8(&_cbs[i].deferred, &expect_true, false) && _cbs[i].callback) {
            _cbs[i].callback(_cbs[i].data);
        }
    }
}

void NINAW132Interface::attach(Callback<void(nsapi_event_t, intptr_t)> status_cb)
{
    _conn_stat_cb = status_cb;
}

nsapi_connection_status_t NINAW132Interface::get_connection_status() const
{
    return _conn_stat;
}

void NINAW132Interface::refresh_conn_state_cb()
{
    nsapi_connection_status_t prev_stat = _conn_stat;
    _conn_stat = _ninaw132.connection_status();

    switch (_conn_stat) {
        // Doesn't require changes
        case NSAPI_STATUS_CONNECTING:
        case NSAPI_STATUS_GLOBAL_UP:
            if (_software_conn_stat == IFACE_STATUS_DISCONNECTED) {
                _software_conn_stat = IFACE_STATUS_CONNECTED;
            }
            break;
        // Start from scratch if connection drops/is dropped
        case NSAPI_STATUS_DISCONNECTED:
            if (_software_conn_stat == IFACE_STATUS_CONNECTED) {
                _software_conn_stat = IFACE_STATUS_DISCONNECTED;
            }
            break;
        // Handled on AT layer
        case NSAPI_STATUS_LOCAL_UP:
        case NSAPI_STATUS_ERROR_UNSUPPORTED:
        default:
            _initialized = false;
            _conn_stat = NSAPI_STATUS_DISCONNECTED;
            for (int i = 0; i < NINAW132_SOCKET_COUNT; i++) {
                _sock_i[i].open = false;
                _sock_i[i].sport = 0;
            }
    }

    if (prev_stat == _conn_stat) {
        return;
    }

    debug_if(_ninaw132_interface_debug, "refresh_conn_state_cb(): Changed to %d.", _conn_stat);

    if (_conn_stat_cb) {
        // _conn_stat_cb will be called in _connect_async or disconnect_assync to avoid race
        // condition
        if ((_software_conn_stat == IFACE_STATUS_CONNECTING
                    || _software_conn_stat == IFACE_STATUS_DISCONNECTING)
                && (_conn_stat != NSAPI_STATUS_CONNECTING)) {
            return;
        }

        _conn_stat_cb(NSAPI_EVENT_CONNECTION_STATUS_CHANGE, _conn_stat);
    }
}

void NINAW132Interface::refresh_socket_data_state_cb(int *sock_id)
{
//     _smutex.lock();
// #if MBED_CONF_RTOS_PRESENT
//     _if_data_available.notify_all();
// #endif
//     _smutex.unlock();

    // not implemented
    if (_socket_stat_cb) {
        _socket_stat_cb(sock_id);
    }
}

void NINAW132Interface::refresh_socket_open_state_cb(int *sock_id)
{
    _smutex.lock();
#if MBED_CONF_RTOS_PRESENT
    _if_socket_opened.notify_all();
#endif
    _smutex.unlock();

    // not implemented
    if (_socket_stat_cb) {
        _socket_stat_cb(sock_id);
    }
}

void NINAW132Interface::proc_oob_evnt()
{
    _oob_event_id = 0; // Allows creation of a new event
    _ninaw132.bg_process_oob(NINA_W132_RECV_TIMEOUT, true);
}

nsapi_error_t NINAW132Interface::_conn_status_to_error()
{
    nsapi_error_t ret;

    switch (_conn_stat) {
        case NSAPI_STATUS_DISCONNECTED:
            ret = NSAPI_ERROR_NO_CONNECTION;
            break;
        case NSAPI_STATUS_CONNECTING:
            ret = NSAPI_ERROR_ALREADY;
            break;
        case NSAPI_STATUS_GLOBAL_UP:
            ret = NSAPI_ERROR_IS_CONNECTED;
            break;
        default:
            ret = NSAPI_ERROR_DEVICE_ERROR;
    }

    return ret;
}

nsapi_error_t NINAW132Interface::set_blocking(bool blocking)
{
    _if_blocking = blocking;

    return NSAPI_ERROR_OK;
}

#if MBED_CONF_NINA_W132_PROVIDE_DEFAULT

WiFiInterface *WiFiInterface::get_default_instance()
{
    static NINAW132Interface ninaw132;
    return &ninaw132;
}

#endif

// #endif
