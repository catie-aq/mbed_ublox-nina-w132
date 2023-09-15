/* NINA-W132 Example
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
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#include <stdlib.h>
#include <string.h>

#include "nina-w132.h"

#define NINA_W132_ALL_SOCKET_IDS -1

#define NINA_W132_DEFAULT_SERIAL_BAUDRATE 115200

using namespace mbed;
using namespace std::chrono;
using std::milli;

// activate / de-activate debug
#define ninaw132_debug 0
#define at_debug 0

NINAW132::NINAW132(PinName tx, PinName rx, PinName resetpin, bool debug):
        _at_v(-1, -1, -1),
        _callback(),
        _serial(tx, rx, MBED_CONF_NINA_W132_SERIAL_BAUDRATE),
        _parser(&_serial),
        _resetpin(resetpin),
        _packets(0),
        _packets_end(&_packets),
        _sock_active_id(-1),
        _heap_usage(0),
        _disconnect(false),
        _conn_status(NSAPI_STATUS_DISCONNECTED)
{
    _wifi_mode = WIFI_MODE_STATION;
    _parser.debug_on(at_debug);
    _parser.set_delimiter("\r\n");
    _parser.oob("+STARTUP", callback(this, &NINAW132::_oob_ready));
    _parser.oob("+UWSCAN:", callback(this, &NINAW132::_oob_scan_results));
    _parser.oob("+UUND:", callback(this, &NINAW132::_oob_disconnection));
    _parser.oob("+UUWLD:", callback(this, &NINAW132::_oob_link_disconnected));
    _parser.oob("+UUWLE:", callback(this, &NINAW132::_oob_connection));

    uart_enable_input(true);

    for (int i = 0; i < SOCKET_COUNT; i++) {
        _sock_i[i].open = false;
        _sock_i[i].proto = NSAPI_UDP;
        _sock_i[i].tcp_data = NULL;
        _sock_i[i].tcp_data_avbl = 0;
        _sock_i[i].tcp_data_rcvd = 0;
        _sock_i[i].send_fail = false;
    }

    _scan_r.res = NULL;
    _scan_r.limit = 0;
    _scan_r.cnt = 0;

    _ninaw132_debug = debug || ninaw132_debug;

    hardware_reset();
}

void NINAW132::hardware_reset()
{
    debug_if(_ninaw132_debug, "NINAW132: Hard reset Module\r\n");
    _resetpin = 0;
    ThisThread::sleep_for(100ms);
    _resetpin = 1;
    ThisThread::sleep_for(1500ms);
}

bool NINAW132::at_available()
{
    bool ready = false;

    _smutex.lock();
    // Might take a while to respond after HW reset
    for (int i = 0; i < 5; i++) {
        ready = _parser.send("AT") && _parser.recv("OK\n");
        if (ready) {
            break;
        }
        debug_if(_ninaw132_debug, "at_available(): Waiting AT response.\r\n");
    }
    // Switch baud-rate from default one to assigned one
    if (MBED_CONF_NINA_W132_SERIAL_BAUDRATE != NINA_W132_DEFAULT_SERIAL_BAUDRATE) {
        ready &= _parser.send("AT+UMRS=%u,2,8,1,1,1", MBED_CONF_NINA_W132_SERIAL_BAUDRATE)
                && _parser.recv("OK\n");
        _serial.set_baud(MBED_CONF_NINA_W132_SERIAL_BAUDRATE);
        ThisThread::sleep_for(50ms);
        ready &= _parser.send("AT") && _parser.recv("OK\n");
    }
    _smutex.unlock();

    return ready;
}

bool NINAW132::echo_off()
{
    _smutex.lock();
    bool ready = _parser.send("ATE0") && _parser.recv("OK\n");
    _smutex.unlock();

    return ready;
}

struct NINAW132::fw_at_version NINAW132::get_firmware_version()
{
    int major;
    int minor;
    int patch;
    int nused;

    _smutex.lock();
    bool done = _parser.send("AT+GMR")
            && _parser.recv("\"%d.%d.%d-%d\"", &major, &minor, &patch, &nused)
            && _parser.recv("OK\n");
    _smutex.unlock();

    if (done) {
        _at_v.major = major;
        _at_v.minor = minor;
        _at_v.patch = patch;
    }
    return _at_v;
}

bool NINAW132::startup(int mode)
{
    bool done = false;
    if (!(mode == (int)WIFI_MODE_STATION || mode == (int)WIFI_MODE_ACCESS_POINT)) {
        return false;
    }

    _smutex.lock();
    set_timeout();
    if (mode == (int)WIFI_MODE_STATION) {
        done = _parser.send("AT+UWSC=0,0,1") && _parser.recv("OK\n");
    } else {
        done = _parser.send("AT+UWAPC=0,0,1") && _parser.recv("OK\n");
    }

    set_timeout(); // Restore default
    _smutex.unlock();

    return done;
}

bool NINAW132::software_reset(void)
{
    // UNSUPPORTED
    return false;
}

bool NINAW132::authentification_type(nsapi_security_t security)
{
    int sec = 1; // Open: default value

    _smutex.lock();

    if (security == NSAPI_SECURITY_WPA || security == NSAPI_SECURITY_WPA2
            || security == NSAPI_SECURITY_WPA3) {
        sec = 2;
    } else if (security == NSAPI_SECURITY_EAP_TLS) {
        sec = 5;
    } else if (security == NSAPI_SECURITY_PEAP) {
        sec = 4;
    }

    bool done = _parser.send("AT+UWSC=0,5,%d", sec) && _parser.recv("OK");

    set_timeout(); // Restore default
    _smutex.unlock();

    return done;
}

bool NINAW132::dhcp(bool enabled)
{
    bool done = false;

    _smutex.lock();

    if (_wifi_mode == WIFI_MODE_STATION) {
        done = _parser.send("AT+UWSC=0,100,%d", enabled ? 2 : 0) && _parser.recv("OK\n");
    } else {
        done = _parser.send("AT+UWAPC=0,106,%d", enabled ? 1 : 0) && _parser.recv("OK\n");
    }

    set_timeout(); // restore default timeout
    _smutex.unlock();

    return done;
}

nsapi_error_t NINAW132::connect(const char *ap, const char *passPhrase, nsapi_security_t ap_sec)
{
    nsapi_error_t ret = NSAPI_ERROR_OK;
    bool res = false;

    _smutex.lock();
    // set authentification type
    if (ap_sec != NSAPI_SECURITY_NONE) {
        res = authentification_type(ap_sec);
        if (!res) {
            return NSAPI_ERROR_AUTH_FAILURE;
        }
    } else {
        return NSAPI_ERROR_PARAMETER;
    }

    // set the SSID
    res = _parser.send("AT+UWSC=0,2,\"%s\"", ap);
    if (!res || !_parser.recv("OK")) {
        debug_if(_ninaw132_debug, "set SSID: parameter error: %s\n", ap);
        ret = NSAPI_ERROR_PARAMETER;
    }
    set_timeout(); // restore default timeout

    // set the password
    if (ret == NSAPI_ERROR_OK) {
        if (passPhrase != NULL) {
            res = _parser.send("AT+UWSC=0,8,\"%s\"", passPhrase);
            if (!res || !_parser.recv("OK")) {
                ret = NSAPI_ERROR_PARAMETER;
                debug_if(_ninaw132_debug, "set SSID: parameter error: %s\n", passPhrase);
            }
        } else {
            ret = NSAPI_ERROR_PARAMETER;
            debug_if(_ninaw132_debug, "set SSID: parameter error: %s\n", passPhrase);
        }
    }

    set_timeout(NINA_W132_CONNECT_TIMEOUT);

    if (ret == NSAPI_ERROR_OK) {
        res = _parser.send("AT+UWSCA=0,3");
        if (!res || !_parser.recv("OK")) {
            ret = NSAPI_ERROR_AUTH_FAILURE;
            debug_if(_ninaw132_debug, "set SSID: parameter error: %s\n");
        }
    }
    set_timeout(); // restore default timeout

    _smutex.unlock();

    return ret;
}

bool NINAW132::disconnect(void)
{
    _smutex.lock();
    _disconnect = true;
    bool done = _parser.send("AT+UWSCAN=0,4") && _parser.recv("OK\n");
    _smutex.unlock();

    return done;
}

bool NINAW132::set_ip_addr(const char *ip, const char *gateway, const char *netmask)
{
    // Unsupported
    return false;
}

const char *NINAW132::getIPAddress(void)
{
    _smutex.lock();
    set_timeout(NINA_W132_CONNECT_TIMEOUT);
    if (!(_parser.send("AT+UNSTAT=0,101") && _parser.recv("+UNSTAT:0,101,%15s\r\n", _ip_buffer)
                && _parser.recv("OK"))) {
        _smutex.unlock();
        return 0;
    }
    set_timeout();
    _smutex.unlock();

    return _ip_buffer;
}

const char *NINAW132::getMACAddress(void)
{
    _smutex.lock();
    if (!(_parser.send("AT+UWAPMACADDR") && _parser.recv("+UWAPMACADDR:%12s\r\n", _mac_buffer)
                && _parser.recv("OK"))) {
        _smutex.unlock();
        return 0;
    }

    set_timeout();
    _smutex.unlock();

    return _mac_buffer;
}

const char *NINAW132::getGateway()
{
    _smutex.lock();
    if (!(_parser.send("AT+UNSTAT=0,103") && _parser.recv("+UNSTAT:0,103,%15s\r\n", _gateway_buffer)
                && _parser.recv("OK"))) {
        _smutex.unlock();
        return 0;
    }
    set_timeout();
    _smutex.unlock();

    return _gateway_buffer;
}

const char *NINAW132::getNetmask()
{
    _smutex.lock();
    if (!(_parser.send("AT+UNSTAT=0,102") && _parser.recv("+UNSTAT:0,102,%15s\r\n", _netmask_buffer)
                && _parser.recv("OK"))) {
        _smutex.unlock();
        return 0;
    }
    set_timeout();
    _smutex.unlock();

    return _netmask_buffer;
}

int8_t NINAW132::getRSSI()
{
    int8_t rssi_value = 0;

    _smutex.lock();
    if (!(_parser.send("AT+UWSSTAT=6") && _parser.recv("+UWSSTAT:6,%d\r\n", &rssi_value)
                && _parser.recv("OK"))) {
        _smutex.unlock();
        return 0;
    }
    set_timeout();
    _smutex.unlock();

    return rssi_value;
}

int NINAW132::scan(WiFiAccessPoint *res, unsigned limit)
{
    _smutex.lock();

    // Default timeout plus time spend scanning each channel
    set_timeout(NINA_W132_SCAN_TIME_MAX_DEFAULT);

    _scan_r.res = res;
    _scan_r.limit = limit;
    _scan_r.cnt = 0;

    bool ret_parse_send = true;

    ret_parse_send = _parser.send("AT+UWSCAN");

    if (!(ret_parse_send && _parser.recv("OK"))) {
        debug_if(_ninaw132_debug, "scan(): AP info parsing aborted.");
        // Lets be happy about partial success and not return NSAPI_ERROR_DEVICE_ERROR
        debug_if(_ninaw132_debug, "scan_r count: %d\n", _scan_r.cnt);
        if (!_scan_r.cnt) {
            _scan_r.cnt = NSAPI_ERROR_DEVICE_ERROR;
        }
    }

    int cnt = _scan_r.cnt;
    _scan_r.res = NULL;

    set_timeout();
    _smutex.unlock();

    return cnt;
}

bool NINAW132::isConnected(void)
{
    return getIPAddress() != 0;
}

nsapi_error_t NINAW132::open_tcp(int id, const char *addr, int port, int keepalive)
{
    static const char *type = "tcp";
    bool done = false;

    if (!addr) {
        return NSAPI_ERROR_PARAMETER;
    }
    _smutex.lock();

    // process OOB so that _sock_i reflects the correct state of the socket
    _process_oob(NINA_W132_SEND_TIMEOUT, true);

    // See the reason above with close()
    if (id >= SOCKET_COUNT) {
        _smutex.unlock();
        return NSAPI_ERROR_PARAMETER;
    } else if (_sock_i[id].open) {
        close(id);
    }

    for (int i = 0; i < 2; i++) {
        if (keepalive) {
            done = _parser.send(
                    "AT+UDCP=\"%s//%s:%d/?flush_tx=2\"", id, type, addr, port);
        } else {
            done = _parser.send("AT+UDCP=\"%s//%s:%d\"", id, type, addr, port);
        }

        if (done) {
            if (!_parser.recv("OK\n")) {
                if (_sock_already) {
                    _sock_already = false; // To be raised again by OOB msg
                    done = close(id);
                    if (!done) {
                        break;
                    }
                }
                if (_error) {
                    _error = false;
                    done = false;
                }
                continue;
            }
            _sock_i[id].open = true;
            _sock_i[id].proto = NSAPI_TCP;
            break;
        }
    }
    _clear_socket_packets(id);

    _smutex.unlock();

    debug_if(_ninaw132_debug,
            "open_tcp: TCP socket %d opened: %s . ",
            id,
            (_sock_i[id].open ? "true" : "false"));

    return done ? NSAPI_ERROR_OK : NSAPI_ERROR_DEVICE_ERROR;
}

bool NINAW132::dns_lookup(const char *name, char *ip)
{
    // TO DO
    // _smutex.lock();
    // set_timeout(NINA_W132_DNS_TIMEOUT);
    // bool done = _parser.send("AT+CIPDOMAIN=\"%s\"", name)
    //         && _parser.recv("+CIPDOMAIN:%15[^\n]\n", ip) && _parser.recv("OK\n");
    // set_timeout();
    // _smutex.unlock();

    return false;
}

nsapi_size_or_error_t NINAW132::send(int id, const void *data, uint32_t amount)
{
    if (_sock_i[id].proto == NSAPI_TCP) {
        if (_sock_sending_id >= 0 && _sock_sending_id < SOCKET_COUNT) {
            if (!_sock_i[id].send_fail) {
                debug_if(_ninaw132_debug,
                        "send(): Previous packet (socket %d) was not yet ACK-ed with SEND OK.",
                        _sock_sending_id);
                return NSAPI_ERROR_WOULD_BLOCK;
            } else {
                debug_if(_ninaw132_debug, "send(): Previous packet (socket %d) failed.", id);
                return NSAPI_ERROR_DEVICE_ERROR;
            }
        }
    }

    nsapi_error_t ret = NSAPI_ERROR_DEVICE_ERROR;
    int bytes_confirmed = 0;

    // +CIPSEND supports up to 2048 bytes at a time
    // Data stream can be truncated
    if (amount > 2048 && _sock_i[id].proto == NSAPI_TCP) {
        amount = 2048;
        // Datagram must stay intact
    } else if (amount > 2048 && _sock_i[id].proto == NSAPI_UDP) {
        debug_if(_ninaw132_debug, "send(): UDP datagram maximum size is 2048 .");
        return NSAPI_ERROR_PARAMETER;
    }

    _smutex.lock();
    // Mark this socket is sending. We allow only one actively sending socket because:
    // 1. NINAW132 AT packets 'SEND OK'/'SEND FAIL' are not associated with socket ID. No way to
    // tell them.
    // 2. In original implementation, NINAW132::send() is synchronous, which implies only one
    // actively sending socket.
    _sock_sending_id = id;
    set_timeout(NINA_W132_SEND_TIMEOUT);
    _busy = false;
    _error = false;
    if (!_parser.send("AT+UDATW=%d,%" PRIu32, id, amount)) {
        debug_if(_ninaw132_debug, "send(): AT+CIPSEND failed.");
        goto END;
    }

    if (!_parser.recv(">")) {
        // This means NINAW132 hasn't even started to receive data
        debug_if(_ninaw132_debug, "send(): Didn't get \">\"");
        if (_sock_i[id].proto == NSAPI_TCP) {
            ret = NSAPI_ERROR_WOULD_BLOCK; // Not necessarily critical error.
        } else if (_sock_i[id].proto == NSAPI_UDP) {
            ret = NSAPI_ERROR_NO_MEMORY;
        }
        goto END;
    }

    if (_parser.write((char *)data, (int)amount) < 0) {
        debug_if(_ninaw132_debug, "send(): Failed to write serial data");
        // Serial is not working, serious error, reset needed.
        ret = NSAPI_ERROR_DEVICE_ERROR;
        goto END;
    }

    // The "Recv X bytes" is not documented.
    if (!_parser.recv("Recv %d bytes", &bytes_confirmed)) {
        debug_if(_ninaw132_debug, "send(): Bytes not confirmed.");
        if (_sock_i[id].proto == NSAPI_TCP) {
            ret = NSAPI_ERROR_WOULD_BLOCK;
        } else if (_sock_i[id].proto == NSAPI_UDP) {
            ret = NSAPI_ERROR_NO_MEMORY;
        }
    } else if (bytes_confirmed != (int)amount && _sock_i[id].proto == NSAPI_UDP) {
        debug_if(_ninaw132_debug,
                "send(): Error: confirmed %d bytes, but expected %d.",
                bytes_confirmed,
                amount);
        ret = NSAPI_ERROR_DEVICE_ERROR;
    } else {
        // TCP can accept partial writes (if they ever happen)
        ret = bytes_confirmed;
    }

END:
    _process_oob(
            NINA_W132_RECV_TIMEOUT, true); // Drain USART receive register to avoid data overrun

    // error hierarchy, from low to high
    // NOTE: We cannot return NSAPI_ERROR_WOULD_BLOCK when "Recv X bytes" has reached, otherwise
    // duplicate data send.
    if (_busy && ret < 0) {
        ret = NSAPI_ERROR_WOULD_BLOCK;
        debug_if(_ninaw132_debug, "send(): Modem busy.");
    }

    if (_error) {
        // FIXME: Not sure clear or not of _error. See it as device error and it can recover only
        // via reset?
        _sock_sending_id = -1;
        ret = NSAPI_ERROR_CONNECTION_LOST;
        debug_if(_ninaw132_debug, "send(): Connection disrupted.");
    }

    if (_sock_i[id].send_fail) {
        _sock_sending_id = -1;
        if (_sock_i[id].proto == NSAPI_TCP) {
            ret = NSAPI_ERROR_DEVICE_ERROR;
        } else {
            ret = NSAPI_ERROR_NO_MEMORY;
        }
        debug_if(_ninaw132_debug, "send(): SEND FAIL received.");
    }

    if (!_sock_i[id].open && ret < 0) {
        _sock_sending_id = -1;
        ret = NSAPI_ERROR_CONNECTION_LOST;
        debug_if(_ninaw132_debug, "send(): Socket %d closed abruptly.", id);
    }

    set_timeout();
    _smutex.unlock();

    return ret;
}

void NINAW132::_oob_packet_hdlr()
{
    int id;
    int port;
    int amount;
    int pdu_len;

    // Get socket id
    if (!_parser.scanf(",%d,", &id)) {
        return;
    }

    if (_tcp_passive && _sock_i[id].open == true && _sock_i[id].proto == NSAPI_TCP) {
        // For TCP +IPD return only id and amount and it is independent on AT+CIPDINFO settings
        // Unfortunately no information about that in ESP manual but it has sense.
        if (_parser.recv("%d\n", &amount)) {
            _sock_i[id].tcp_data_avbl = amount;

            // notify data is available
            if (_callback) {
                _callback();
            }
        }
        return;
    } else {
        if (!(_parser.scanf("%d,", &amount) && _parser.scanf("%15[^,],", _ip_buffer)
                    && _parser.scanf("%d:", &port))) {
            return;
        }
    }

    pdu_len = sizeof(struct packet) + amount;

    if ((_heap_usage + pdu_len) > MBED_CONF_NINA_W132_SOCKET_BUFSIZE) {
        debug_if(_ninaw132_debug, "\"nina-w132.socket-bufsize\"-limit exceeded, packet dropped");
        return;
    }

    struct packet *packet = (struct packet *)malloc(pdu_len);
    if (!packet) {
        debug_if(_ninaw132_debug,
                "_oob_packet_hdlr(): Out of memory, unable to allocate memory for packet.");
        return;
    }
    _heap_usage += pdu_len;

    packet->id = id;
    if (_sock_i[id].proto == NSAPI_UDP) {
        packet->remote_port = port;
        memcpy(packet->remote_ip, _ip_buffer, 16);
    }
    packet->len = amount;
    packet->alloc_len = amount;
    packet->next = 0;

    if (_parser.read((char *)(packet + 1), amount) < amount) {
        free(packet);
        _heap_usage -= pdu_len;
        return;
    }

    // append to packet list
    *_packets_end = packet;
    _packets_end = &packet->next;
}

void NINAW132::_process_oob(duration<uint32_t, milli> timeout, bool all)
{
    set_timeout(timeout);
    // Poll for inbound packets
    while (_parser.process_oob() && all) { }
    set_timeout();
}

void NINAW132::bg_process_oob(duration<uint32_t, milli> timeout, bool all)
{
    _smutex.lock();
    _process_oob(timeout, all);
    _smutex.unlock();
}

int32_t NINAW132::_recv_tcp_passive(
        int id, void *data, uint32_t amount, duration<uint32_t, milli> timeout)
{
    int32_t ret = NSAPI_ERROR_WOULD_BLOCK;

    _smutex.lock();

    _process_oob(timeout, true);

    if (_sock_i[id].tcp_data_avbl != 0) {
        _sock_i[id].tcp_data = (char *)data;
        _sock_i[id].tcp_data_rcvd = NSAPI_ERROR_WOULD_BLOCK;
        _sock_active_id = id;

        // +CIPRECVDATA supports up to 2048 bytes at a time
        amount = amount > 2048 ? 2048 : amount;

        // NOTE: documentation v3.0 says '+CIPRECVDATA:<data_len>,' but it's not how the FW
        // responds...
        bool done = _parser.send("AT+CIPRECVDATA=%d,%" PRIu32, id, amount) && _parser.recv("OK\n");

        _sock_i[id].tcp_data = NULL;
        _sock_active_id = -1;

        if (!done) {
            goto BUSY;
        }

        // update internal variable tcp_data_avbl to reflect the remaining data
        if (_sock_i[id].tcp_data_rcvd > 0) {
            if (_sock_i[id].tcp_data_rcvd > (int32_t)amount) {
                MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER, MBED_ERROR_CODE_EBADMSG),
                        "NINAW132::_recv_tcp_passive() too much data from modem\n");
            }
            if (_sock_i[id].tcp_data_avbl > _sock_i[id].tcp_data_rcvd) {
                _sock_i[id].tcp_data_avbl -= _sock_i[id].tcp_data_rcvd;
            } else {
                _sock_i[id].tcp_data_avbl = 0;
            }
        }

        ret = _sock_i[id].tcp_data_rcvd;
    }

    if (!_sock_i[id].open && ret == NSAPI_ERROR_WOULD_BLOCK) {
        ret = 0;
    }

    _smutex.unlock();
    return ret;

BUSY:
    _process_oob(NINA_W132_RECV_TIMEOUT, true);
    if (_busy) {
        debug_if(_ninaw132_debug, "_recv_tcp_passive(): Modem busy.");
        ret = NSAPI_ERROR_WOULD_BLOCK;
    } else {
        debug_if(_ninaw132_debug, "_recv_tcp_passive(): Unknown state.");
        ret = NSAPI_ERROR_DEVICE_ERROR;
    }
    _smutex.unlock();
    return ret;
}

int32_t NINAW132::recv_tcp(int id, void *data, uint32_t amount, duration<uint32_t, milli> timeout)
{
    if (_tcp_passive) {
        return _recv_tcp_passive(id, data, amount, timeout);
    }

    _smutex.lock();

    // check if any packets are ready for us
    for (struct packet **p = &_packets; *p; p = &(*p)->next) {
        if ((*p)->id == id) {
            struct packet *q = *p;

            if (q->len <= amount) { // Return and remove full packet
                memcpy(data, q + 1, q->len);

                if (_packets_end == &(*p)->next) {
                    _packets_end = p;
                }
                *p = (*p)->next;

                _smutex.unlock();

                uint32_t pdu_len = sizeof(struct packet) + q->alloc_len;
                uint32_t len = q->len;
                free(q);
                _heap_usage -= pdu_len;
                return len;
            } else { // return only partial packet
                memcpy(data, q + 1, amount);

                q->len -= amount;
                memmove(q + 1, (uint8_t *)(q + 1) + amount, q->len);

                _smutex.unlock();
                return amount;
            }
        }
    }
    if (!_sock_i[id].open) {
        _smutex.unlock();
        return 0;
    }

    _smutex.unlock();

    return NSAPI_ERROR_WOULD_BLOCK;
}

void NINAW132::_clear_socket_packets(int id)
{
    struct packet **p = &_packets;

    while (*p) {
        if ((*p)->id == id || id == NINA_W132_ALL_SOCKET_IDS) {
            struct packet *q = *p;
            int pdu_len = sizeof(struct packet) + q->alloc_len;

            if (_packets_end == &(*p)->next) {
                _packets_end = p; // Set last packet next field/_packets
            }
            *p = (*p)->next;
            free(q);
            _heap_usage -= pdu_len;
        } else {
            // Point to last packet next field
            p = &(*p)->next;
        }
    }
    if (id == NINA_W132_ALL_SOCKET_IDS) {
        for (int id = 0; id < 5; id++) {
            _sock_i[id].tcp_data_avbl = 0;
        }
    } else {
        _sock_i[id].tcp_data_avbl = 0;
    }
}

void NINAW132::_clear_socket_sending(int id)
{
    if (id == _sock_sending_id) {
        _sock_sending_id = -1;
    }
    _sock_i[id].send_fail = false;
}

bool NINAW132::close(int id)
{
    // May take a second try if device is busy
    for (unsigned i = 0; i < 2; i++) {
        _smutex.lock();
        if (_parser.send("AT+CIPCLOSE=%d", id)) {
            if (!_parser.recv("OK\n")) {
                if (_closed) { // UNLINK ERROR
                    _closed = false;
                    _sock_i[id].open = false;
                    _clear_socket_packets(id);
                    // Closed, so this socket escapes from SEND FAIL status.
                    _clear_socket_sending(id);
                    _smutex.unlock();
                    // NINAW132 has a habit that it might close a socket on its own.
                    debug_if(_ninaw132_debug, "close(%d): socket close OK with UNLINK ERROR", id);
                    return true;
                }
            } else {
                // _sock_i[id].open set to false with an OOB
                _clear_socket_packets(id);
                // Closed, so this socket escapes from SEND FAIL status
                _clear_socket_sending(id);
                _smutex.unlock();
                debug_if(_ninaw132_debug, "close(%d): socket close OK with AT+CIPCLOSE OK", id);
                return true;
            }
        }
        _smutex.unlock();
    }

    debug_if(_ninaw132_debug, "close(%d): socket close FAIL'ed (spurious close)", id);
    return false;
}

void NINAW132::set_timeout(duration<uint32_t, milli> timeout)
{
    _parser.set_timeout(timeout.count());
}

bool NINAW132::readable()
{
    return _serial.FileHandle::readable();
}

bool NINAW132::writeable()
{
    return _serial.FileHandle::writable();
}

void NINAW132::sigio(Callback<void()> func)
{
    _serial.sigio(func);
    _callback = func;
}

void NINAW132::attach(Callback<void()> status_cb)
{
    _conn_stat_cb = status_cb;
}

bool NINAW132::_recv_ap(nsapi_wifi_ap_t *ap)
{
    uint8_t sec = NSAPI_SECURITY_UNKNOWN;
    int ret;

    char line_buffer[70]; // Buffer length needs to be refined

    // Scanf the entire line into a buffer for further processing
    ret = _parser.scanf("%s\n", line_buffer);

    if (ret < 0) {
        fflush(stdout);
        _parser.abort();
        debug_if(_ninaw132_debug, "_recv_ap(): scanf error.");
    }

    // Initialize the SSID to all \0
    // Scanf the buffer to scrape all appropriate datas
    ret = sscanf(line_buffer,
            "%2hhX%2hhX%2hhX%2hhX%2hhX%2hhX,%*d,\"%32[^\"]\" ,%hhu,%hhd,%hhu,%*d,%*d",
            &ap->bssid[0],
            &ap->bssid[1],
            &ap->bssid[2],
            &ap->bssid[3],
            &ap->bssid[4],
            &ap->bssid[5],
            ap->ssid,
            &ap->channel,
            &ap->rssi,
            &sec);

    if (ret < 9) {
        // Scanf the buffer again if no SSID is detected (scanf error)
        ret = sscanf(line_buffer,
                "%2hhX%2hhX%2hhX%2hhX%2hhX%2hhX,%*d,\"\" ,%hhu,%hhd,%hhu,%*d,%*d",
                &ap->bssid[0],
                &ap->bssid[1],
                &ap->bssid[2],
                &ap->bssid[3],
                &ap->bssid[4],
                &ap->bssid[5],
                &ap->channel,
                &ap->rssi,
                &sec);

        sprintf(ap->ssid, "(hidden)");
    }

    if (sec == 0x00)
        ap->security = NSAPI_SECURITY_NONE;
    else if ((sec & 0x08) == 0x08)
        ap->security = NSAPI_SECURITY_WPA;
    else if ((sec & 0x10) == 0x10)
        ap->security = NSAPI_SECURITY_WPA2;
    else if ((sec & 0x18) == 0x18)
        ap->security = NSAPI_SECURITY_WPA_WPA2;
    else if ((sec & 0x18) == 0x20)
        ap->security = NSAPI_SECURITY_WPA3;
    else if ((sec & 0x18) == 0x30)
        ap->security = NSAPI_SECURITY_WPA3_WPA2;
    else
        ap->security = NSAPI_SECURITY_UNKNOWN;

    return ret < 0 ? false : true;
}

void NINAW132::_oob_ready()
{
    debug_if(_ninaw132_debug, "ready to use AT commands.");
}

void NINAW132::_oob_tcp_data_hdlr()
{
    int32_t len;

    MBED_ASSERT(_sock_active_id >= 0 && _sock_active_id < 5);

    if (!_parser.scanf("%" SCNd32 ":", &len)) {
        return;
    }

    if (_parser.read(_sock_i[_sock_active_id].tcp_data, len) == -1) {
        return;
    }

    _sock_i[_sock_active_id].tcp_data_rcvd = len;
}

void NINAW132::_oob_scan_results()
{
    nsapi_wifi_ap_t ap;

    if (_recv_ap(&ap)) {
        if (_scan_r.res && _scan_r.cnt < _scan_r.limit) {
            _scan_r.res[_scan_r.cnt] = WiFiAccessPoint(ap);
        }

        _scan_r.cnt++;
    }
}

void NINAW132::_oob_connection()
{
    // Ignore the session ID, it's not relevant

    _conn_status = NSAPI_STATUS_CONNECTING;
    MBED_ASSERT(_conn_stat_cb);
    _conn_stat_cb();
}

void NINAW132::_oob_disconnection()
{
    _conn_status = NSAPI_STATUS_DISCONNECTED;
    _disconnect = false;

    MBED_ASSERT(_conn_stat_cb);
    _conn_stat_cb();
}

void NINAW132::_oob_link_disconnected()
{
    // TODO
    int reason = 0;
    int connection_id = 0;

    if (_parser.recv("%d, %d\n", &connection_id, &reason)) {
        _disconnection_reason = reason;
        if (reason == SECURITY_PROBLEM) {
            // force disconnection
            disconnect();
            return;
        }
    }
}

void NINAW132::flush()
{
    _smutex.lock();
    _parser.flush();
    _smutex.unlock();
}

nsapi_connection_status_t NINAW132::connection_status() const
{
    return _conn_status;
}

int NINAW132::uart_enable_input(bool enabled)
{
    return _serial.enable_input(enabled);
}

// #endif
