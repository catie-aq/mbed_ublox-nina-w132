/* NINA_W132 Example
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

#ifndef NINA_W132_H
#define NINA_W132_H

// #if DEVICE_SERIAL && DEVICE_INTERRUPTIN && defined(MBED_CONF_EVENTS_PRESENT) &&
// defined(MBED_CONF_NSAPI_PRESENT) && defined(MBED_CONF_RTOS_API_PRESENT)
#include <ctime>
#include <stdint.h>

#include "mbed.h"
#include "platform/mbed_chrono.h"

// Various timeouts for different NINA_W132 operations
// (some of these can't use literal form as they're needed for defaults in this header, where
// we shouldn't add a using directive for them. Defines only used in the C++ file can use literals).
#ifndef NINA_W132_CONNECT_TIMEOUT
#define NINA_W132_CONNECT_TIMEOUT 15s
#endif
#ifndef NINA_W132_OPEN_TIMEOUT
#define NINA_W132_OPEN_TIMEOUT 5s
#endif
#ifndef NINA_W132_SEND_TIMEOUT
#define NINA_W132_SEND_TIMEOUT 2s
#endif
#ifndef NINA_W132_RECV_TIMEOUT
#define NINA_W132_RECV_TIMEOUT 10s
#endif
#ifndef NINA_W132_MISC_TIMEOUT
#define NINA_W132_MISC_TIMEOUT std::chrono::seconds(2)
#endif
#ifndef NINA_W132_DNS_TIMEOUT
#define NINA_W132_DNS_TIMEOUT 15s
#endif

#define NINA_W132_SCAN_TIME_MAX_DEFAULT std::chrono::seconds(2)

struct nina_w132_socket {
    int id;
    nsapi_protocol_t proto;
    bool connected;
    bool bound;
    SocketAddress addr;
    int keepalive; // TCP
};

/** NINA_W132Interface class.
    This is an interface to a NINA_W132 radio.
 */
class NINAW132 {
public:
    // wifi mode
    static const int8_t WIFI_MODE_STATION = 1;
    static const int8_t WIFI_MODE_ACCESS_POINT = 2;
    // TCP UDP data format
    static const uint8_t TCP_UDP_STRING_DATA_FORMAT = 0;
    static const uint8_t TCP_UDP_HEXA_DATA_FORMAT = 1;
    static const uint8_t TCP_UDP_BINARY_DATA_FORMAT = 2;

    enum disconnection_reason {
        UNKNOWN = 0,
        REMOTE_CLOSE = 1,
        OUT_OF_RANGE = 2,
        ROAMING = 3,
        SECURITY_PROBLEM = 4,
        NETWORK_DISABLE = 5
    };

    NINAW132(PinName tx, PinName rx, PinName resetpin = NC, bool debug = false);

    /**
     * NINAW132 firmware AT version
     *
     * @param major Major version number
     * @param minor Minor version number
     * @param patch Patch version number
     */
    struct fw_at_version {
        int major;
        int minor;
        int patch;

        fw_at_version(int major, int minor, int patch): major(major), minor(minor), patch(patch)
        {
        }
    };

    /**
     * Check AT command interface of NINAW132
     *
     * @return true if ready to respond on AT commands
     */
    bool at_available(void);

    /**
     * Disable echo - required for OOB processing to work
     *
     * @return true if echo was successfully disabled
     */
    bool echo_off(void);

    /**
     * Check AT instruction set version from which firmware is created
     *
     * @return fw_at_version which tells major, minor and patch version
     */
    struct fw_at_version get_firmware_version(void);

    /**
     * Startup the NINAW132
     *
     * @param mode mode of WIFI 1-client, 2-host, 3-both
     * @return true only if NINAW132 was setup correctly
     */
    bool startup(int mode);

    /**
     * Hardware reset NINAW132
     *
     * @return true only if NINAW132 resets successfully
     */
    void hardware_reset(void);

    /**
     * Software reset NINAW132
     *
     * @return true only if NINAW132 resets successfully
     */
    bool software_reset(void);

    /**
     * Enable/Disable DHCP
     *
     * @param enabled DHCP enabled when true
     * @param mode mode of DHCP 0-softAP, 1-station, 2-both
     * @return true only if NINAW132 enables/disables DHCP successfully
     */
    bool dhcp(bool enabled);

    /**
     * authentification type NINAW132 to AP
     *
     * @param security the security type
     * @return true in success, false in failure
     */
    bool authentification_type(nsapi_security_t security);

    /**
     * Connect NINAW132 to AP
     *
     * @param ap the name of the AP
     * @param passPhrase the password of AP
     * @param ap_sec the security level of network AP
     * @return nsapi_error enum
     */
    int connect(const char *ap, const char *passPhrase, nsapi_security_t ap_sec);

    /**
     * Disconnect NINAW132 from AP
     *
     * @return true only if NINAW132 is disconnected successfully
     */
    bool disconnect(void);

    /**
     * Set static IP address, gateway and netmask
     *
     * @param ip IP address to set
     * @param gateway (optional) gateway to set
     * @param netmask (optional) netmask to set
     *
     * @return true if operation was successful and flase otherwise
     */
    bool set_ip_addr(const char *ip, const char *gateway, const char *netmask);

    /**
     * Get the IP address of NINAW132
     *
     * @return null-teriminated IP address or null if no IP address is assigned
     */
    const char *getIPAddress(void);

    /**
     * Get the MAC address of NINAW132
     *
     * @return null-terminated MAC address or null if no MAC address is assigned
     */
    const char *getMACAddress(void);

    /** Get the local gateway
     *
     *  @return         Null-terminated representation of the local gateway
     *                  or null if no network mask has been recieved
     */
    const char *getGateway();

    /** Get the local network mask
     *
     *  @return         Null-terminated representation of the local network mask
     *                  or null if no network mask has been recieved
     */
    const char *getNetmask();

    /* Return RSSI for active connection
     *
     * @return      Measured RSSI
     */
    int8_t getRSSI();

    /**
     * Check if NINAW132 is conenected
     *
     * @return true only if the chip has an IP address
     */
    bool isConnected(void);

    /** Scan for available networks
     *
     * @param  ap    Pointer to allocated array to store discovered AP
     * @param  limit Size of allocated @a res array, or 0 to only count available AP
     * @return       Number of entries in @a res, or if @a count was 0 number of available networks,
     * negative on error see @a nsapi_error
     */
    int scan(WiFiAccessPoint *res, unsigned limit);

    /**
     * Open a socketed connection
     *
     * @param type the type of socket to open "UDP" or "TCP"
     * @param id id to give the new socket, valid 0-4
     * @param port port to open connection with
     * @param addr the IP address of the destination
     * @param port the port on the destination
     * @param keepalive TCP connection's keep alive time, zero means disabled
     * @return NSAPI_ERROR_OK in success, negative error code in failure
     */
    nsapi_error_t open_tcp(int id, const char *addr, int port, int keepalive = 0);

    /**
     * Open a socketed connection
     *
     * @param type the type of socket to open "UDP" or "TCP"
     * @param id id to give the new socket, valid 0-4
     * @param port port to open connection with
     * @param addr the IP address of the destination
     * @param port the port on the destination
     * @return NSAPI_ERROR_OK in success, negative error code in failure
     */
    nsapi_error_t open_udp(int id, const char *addr, int port);

    /**
     * Sends data to an open UDP socket
     *
     * @param id id of socket to send to
     * @param data data to be sent
     * @param amount amount of data to be sent - max 2000 in the binary data format
     * @return number of bytes on success, negative error code in failure
     */
    nsapi_size_or_error_t send_udp(int id, const void *data, uint32_t amount);

    /**
     * Sends data to an open TCP socket
     *
     * @param id id of socket to send to
     * @param data data to be sent
     * @param amount amount of data to be sent - max 2000 in the binary data format
     * @return number of bytes on success, negative error code in failure
     */
    nsapi_size_or_error_t send_tcp(int id, const void *data, uint32_t amount);

    /**
     * Receives stream data from an open TCP socket
     *
     * @param id id to receive from
     * @param data placeholder for returned information
     * @param amount number of bytes to be received
     * @return the number of bytes received
     */
    int32_t recv_tcp(int id,
            void *data,
            uint32_t amount,
            mbed::chrono::milliseconds_u32 timeout = NINA_W132_RECV_TIMEOUT);

    /**
     * Receives datagram from an open UDP socket
     *
     * @param id id to receive from
     * @param data placeholder for returned information
     * @param amount number of bytes to be received
     * @return the number of bytes received
     */
    int32_t recv_udp(int id,
            void *data,
            uint32_t amount,
            mbed::chrono::milliseconds_u32 timeout = NINA_W132_RECV_TIMEOUT);

    /**
     * Closes a socket
     *
     * @param id id of socket to close, valid only 0-4
     * @return true only if socket is closed successfully
     */
    bool close(int id);

    /**
     * Socket data available
     *
     * @return true if data are available
     */
    bool data_available(int socket_id);

    /**
     * Allows timeout to be changed between commands
     *
     * @param timeout_ms timeout of the connection
     */
    void set_timeout(mbed::chrono::milliseconds_u32 timeout = NINA_W132_MISC_TIMEOUT);

    /**
     * Checks if data is available
     */
    bool readable();

    /**
     * Checks if data can be written
     */
    bool writeable();

    /**
     * Attach a function to call whenever sigio happens in the serial
     *
     * @param func A pointer to a void function, or 0 to set as none
     */
    void sigio(mbed::Callback<void()> func);

    /**
     * Attach a function to call whenever sigio happens in the serial
     *
     * @param obj pointer to the object to call the member function on
     * @param method pointer to the member function to call
     */
    template <typename T, typename M> void sigio(T *obj, M method)
    {
        sigio(mbed::Callback<void()>(obj, method));
    }

    /**
     * Attach a function to call whenever network state has changed.
     *
     * @param func A pointer to a void function, or 0 to set as none
     */
    void attach(mbed::Callback<void()> status_cb);

    template <typename T, typename M> void attach(T *obj, M method)
    {
        attach(mbed::Callback<void()>(obj, method));
    }

    /**
     * Attach a function to call whenever socket receive data state has changed.
     *
     * @param func A pointer to a void function, or 0 to set as none
     */
    void attach_socket_recv(Callback<void(int *current_socket_id)> socket_recv_cb);

    template <typename T, typename M> void attach_socket_recv(T *obj, M method)
    {
        attach_socket_recv(mbed::Callback<void(int *current_socket_id)>(obj, method));
    }

    /**
     * Attach a function to call whenever socket open state has changed.
     *
     * @param func A pointer to a void function, or 0 to set as none
     */
    void attach_socket_open(Callback<void(int *current_socket_id)> socket_open_cb);

    template <typename T, typename M> void attach_socket_open(T *obj, M method)
    {
        attach_socket_open(mbed::Callback<void(int *current_socket_id)>(obj, method));
    }

    /** Get the connection status
     *
     *  @return         The connection status according to ConnectionStatusType
     */
    nsapi_connection_status_t connection_status() const;

    /**
     * For executing OOB processing on background
     *
     * @param timeout AT parser receive timeout
     * @param if TRUE, process all OOBs instead of only one
     */
    void bg_process_oob(std::chrono::duration<uint32_t, std::milli> timeout, bool all);

    /**
     * Flush the serial port input buffers.
     *
     * If you do HW reset for ESP module, you should
     * flush the input buffers from existing responses
     * from the device.
     */
    void flush();

    static const int8_t WIFIMODE_STATION = 1;
    static const int8_t WIFIMODE_ACCESS_POINT = 2;
    static const int8_t SOCKET_COUNT = 5;

    /**
     * Enables or disables uart input and deep sleep
     *
     * @param lock if TRUE, uart input is enabled and  deep sleep is locked
     * if FALSE, uart input is disabled and  deep sleep is unlocked
     */
    int uart_enable_input(bool lock);

private:
    // FW version
    struct fw_at_version _at_v;

    // debug
    bool _ninaw132_debug;

    // FW version specific settings and functionalities
    bool _udp_passive;
    bool _tcp_passive;
    uint8_t _udp_data_format;
    uint8_t _tcp_data_format;
    int32_t _at_tcp_data_recv(int id,
            void *data,
            uint32_t amount,
            std::chrono::duration<uint32_t, std::milli> timeout);
    int32_t _at_udp_data_recv(int id,
            void *data,
            uint32_t amount,
            std::chrono::duration<uint32_t, std::milli> timeout);
    Callback<void()> _callback;

    // UART settings
    BufferedSerial _serial;
    Mutex _smutex; // Protect serial port access

    // AT Command Parser
    ATCmdParser _parser;

    // Reset pin
    DigitalOut _resetpin;

    // Wifi scan result handling
    bool _recv_ap(nsapi_wifi_ap_t *ap);

    // Socket data buffer
    struct packet {
        struct packet *next;
        int id;
        char remote_ip[16];
        int remote_port;
        uint32_t len; // Remaining length
        uint32_t alloc_len; // Original length
        // data follows
    } * _packets, **_packets_end;

    void _clear_socket_packets(int id);
    void _clear_socket_sending(int id);
    int _sock_active_id;
    mbed::Callback<void(int *current_socket_id)> _socket_recv_cb; // NINAW132Interface registered
    mbed::Callback<void(int *current_socket_id)> _socket_conn_cb; // NINAW132Interface registered

#if MBED_CONF_RTOS_PRESENT
    rtos::ConditionVariable _if_data_available;
#endif

    // Memory statistics
    size_t _heap_usage; // (Socket data buffer usage)

    // OOB processing
    void _process_oob(std::chrono::duration<uint32_t, std::milli> timeout, bool all);

    // OOB message handlers
    void _oob_socket_disconnection();
    void _oob_socket_connection();
    void _oob_tcp_data_hdlr();
    void _oob_ready();
    void _oob_scan_results();
    void _oob_connected();
    void _oob_disconnection();
    void _oob_link_disconnected();

    // OOB state variables
    int _connect_error;
    bool _disconnect;
    bool _fail;
    bool _sock_already;
    bool _closed;
    bool _error;
    bool _busy;

    // Modem's address info
    char _ip_buffer[16];
    char _gateway_buffer[16];
    char _netmask_buffer[16];
    char _mac_buffer[18];
    uint8_t _wifi_mode;

    // Modem's socket info
    struct _sock_info {
        bool open;
        nsapi_protocol_t proto;
        char *tcp_data;
        bool tcp_data_avbl; // Data waiting on modem
        uint16_t len_tcp_data_rcvd;
        bool send_fail; // Received 'SEND FAIL'. Expect user will close the socket.
    };
    struct _sock_info _sock_i[SOCKET_COUNT];

    // Scan results
    struct _scan_results {
        WiFiAccessPoint *res;
        unsigned limit;
        unsigned cnt;
    };
    struct _scan_results _scan_r;

    // Connection state reporting
    uint16_t _disconnection_reason;
    nsapi_connection_status_t _conn_status;
    mbed::Callback<void()> _conn_stat_cb; // NINAW132Interface registered
};
#endif
// #endif
