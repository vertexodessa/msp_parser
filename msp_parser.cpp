#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <vector>
#include <stdexcept>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

/******************************************************************************
 *                            Constants & Enums
 ******************************************************************************/
static const int  MSP_MAX_PAYLOAD_SIZE = 256;
static const int  FRAME_BUFFER_SIZE    = 1024;
static const int  CHANNEL_COUNT        = 18;

/**
 * Known MSP commands as a strongly typed enum.
 */
enum class MspCommand : uint8_t {
    STATUS     = 101,
    ATTITUDE   = 108,
    RC         = 105,
    FC_VARIANT = 102,
    UNKNOWN    = 255 // fallback
};

/******************************************************************************
 *                           Data Model (Shared State)
 *
 * This struct holds data from the flight controller.
 * Nothing else manipulates how it's stored or interpreted (besides executors).
 ******************************************************************************/
struct FlightDataModel {
    bool     armed               { false };
    int16_t  pitch               { 0 };
    int16_t  roll                { 0 };
    int16_t  heading             { 0 };
    uint16_t channels[CHANNEL_COUNT] {};
    char     fcIdentifier[5]     {};  // 4 chars + null terminator

    uint8_t  frameBuffer[FRAME_BUFFER_SIZE] {};
    size_t   fbCursor           { 0 };
    bool     verbose            { true };

    /**
     * Flush the internal frame buffer.
     */
    void flushFrameBuffer() {
        if (verbose) {
            cout << "[FlightDataModel] Flushing frame buffer of size " << fbCursor << "\n";
        }
        // If you need real sendto(...) calls, do them here
        fbCursor = 0;
    }

    /**
     * Copy arbitrary data to our frame buffer.
     */
    void copyToFrameBuffer(const void* data, size_t size) {
        if ((fbCursor + size) > FRAME_BUFFER_SIZE) {
            flushFrameBuffer();
        }
        memcpy(frameBuffer + fbCursor, data, size);
        fbCursor += size;
    }
};

/******************************************************************************
 *                         MSP Message Representation
 ******************************************************************************/
struct MspMessage {
    enum class Direction : uint8_t {
        OUTBOUND,
        INBOUND
    };

    Direction direction { Direction::OUTBOUND };
    MspCommand cmd      { MspCommand::UNKNOWN };
    uint8_t    size     { 0 };
    uint8_t    checksum { 0 };
    uint8_t    payload[MSP_MAX_PAYLOAD_SIZE] {};
};

/******************************************************************************
 *                            Handler Interface
 ******************************************************************************/
class IMspMessageHandler {
public:
    virtual ~IMspMessageHandler() = default;
    virtual void onMspMessage(const MspMessage& msg) = 0;
};

/**
 * Interface for MSP command executors.
 * Each executor handles exactly one type of command (Single Responsibility).
 */
class IMspCommandExecutor {
public:
    virtual ~IMspCommandExecutor() = default;
    virtual void execute(const MspMessage& msg, FlightDataModel& dataModel) = 0;
};

/******************************************************************************
 *                           MSP Message Parser
 *
 * Receives raw bytes, reconstructs MspMessage objects, and notifies a handler
 * when a message is fully parsed.
 ******************************************************************************/
class MspMessageParser {
public:
    explicit MspMessageParser(IMspMessageHandler& handler)
        : m_handler(handler)
    {
        reset();
    }

    /**
     * Possible states of the MSP parser state machine.
     */
    enum class ParserState {
        IDLE,
        VERSION,
        DIRECTION,
        SIZE,
        CMD,
        PAYLOAD,
        CHECKSUM
    };

    /**
     * Processes a single byte from the input stream.
     */
    void processByte(uint8_t byte) {
        switch (m_state) {
        case ParserState::IDLE:
            if (byte == '$') {
                m_state = ParserState::VERSION;
            }
            break;

        case ParserState::VERSION:
            if (byte == 'M') {
                m_state = ParserState::DIRECTION;
            } else {
                reset();
            }
            break;

        case ParserState::DIRECTION:
            if (byte == '<') {
                m_msg.direction = MspMessage::Direction::OUTBOUND;
            } else if (byte == '>') {
                m_msg.direction = MspMessage::Direction::INBOUND;
            } else {
                reset();
                break;
            }
            m_state = ParserState::SIZE;
            break;

        case ParserState::SIZE:
            m_msg.size     = byte;
            m_msg.checksum = byte; // initial checksum includes size
            m_msg.cmd      = MspCommand::UNKNOWN;
            m_bufPtr       = 0;
            if (m_msg.size > MSP_MAX_PAYLOAD_SIZE) {
                reset();
            } else {
                m_state = ParserState::CMD;
            }
            break;

        case ParserState::CMD:
            m_msg.checksum ^= byte;
            m_msg.cmd       = toMspCommand(byte);
            m_bufPtr        = 0;
            // If size == 0, we move directly to CHECKSUM
            m_state = (m_msg.size == 0) ? ParserState::CHECKSUM : ParserState::PAYLOAD;
            break;

        case ParserState::PAYLOAD:
            m_msg.payload[m_bufPtr++] = byte;
            m_msg.checksum           ^= byte;
            if (m_bufPtr == m_msg.size) {
                m_state = ParserState::CHECKSUM;
            }
            break;

        case ParserState::CHECKSUM:
            if (m_msg.checksum == byte) {
                // Valid message
                m_handler.onMspMessage(m_msg);
            }
            reset(); // Reset regardless
            break;
        }
    }

private:
    IMspMessageHandler& m_handler;
    ParserState         m_state { ParserState::IDLE };
    MspMessage          m_msg   {};
    uint16_t            m_bufPtr{ 0 };

    /**
     * Reset the parser to IDLE state.
     */
    void reset() {
        m_state = ParserState::IDLE;
        memset(&m_msg, 0, sizeof(m_msg));
        m_msg.cmd = MspCommand::UNKNOWN;
        m_bufPtr  = 0;
    }

    /**
     * Convert an integer to our MspCommand enum.
     */
    MspCommand toMspCommand(uint8_t cmd) {
        switch (cmd) {
        case 101: return MspCommand::STATUS;
        case 108: return MspCommand::ATTITUDE;
        case 105: return MspCommand::RC;
        case 102: return MspCommand::FC_VARIANT;
        default:  return MspCommand::UNKNOWN;
        }
    }
};

/******************************************************************************
 *                      Concrete Command Executors
 *
 * Each executor handles a specific MSP command. Extending behavior for new
 * commands simply requires adding a new executor class implementing
 * IMspCommandExecutor.
 ******************************************************************************/

/**
 * MSP 101 -> STATUS
 */
class StatusCommandExecutor : public IMspCommandExecutor {
public:
    void execute(const MspMessage& msg, FlightDataModel& dataModel) override {
        // Typically, payload[6] bit 0 indicates "armed" in Betaflight
        if (msg.size > 6) {
            dataModel.armed = (msg.payload[6] & 0x01);
        }
        if (dataModel.verbose) {
            cout << "[StatusCommandExecutor] Armed = " << dataModel.armed << "\n";
        }
    }
};

/**
 * MSP 108 -> ATTITUDE
 */
class AttitudeCommandExecutor : public IMspCommandExecutor {
public:
    void execute(const MspMessage& msg, FlightDataModel& dataModel) override {
        if (msg.size >= 6) {
            dataModel.roll    = *reinterpret_cast<const int16_t*>(&msg.payload[0]);
            dataModel.pitch   = *reinterpret_cast<const int16_t*>(&msg.payload[2]);
            dataModel.heading = *reinterpret_cast<const int16_t*>(&msg.payload[4]);
            if (dataModel.verbose) {
                cout << "[AttitudeCommandExecutor] pitch:"  << dataModel.pitch
                     << " roll:"    << dataModel.roll
                     << " heading:" << dataModel.heading << endl;
            }
        }
    }
};

/**
 * MSP 102 -> FC_VARIANT
 */
class FcVariantCommandExecutor : public IMspCommandExecutor {
public:
    void execute(const MspMessage& msg, FlightDataModel& dataModel) override {
        // Usually the first 4 bytes represent the FC variant
        if (msg.size >= 4) {
            // Update only if changed
            if (strncmp(dataModel.fcIdentifier,
                        reinterpret_cast<const char*>(msg.payload),
                        4) != 0)
            {
                memcpy(dataModel.fcIdentifier, msg.payload, 4);
                dataModel.fcIdentifier[4] = '\0';
                if (dataModel.verbose) {
                    cout << "[FcVariantCommandExecutor] Flight Controller: "
                         << dataModel.fcIdentifier << "\n";
                }
            }
        }
    }
};

/**
 * We can chain multiple executors for MSP 105 -> RC.
 * 1) RcCommandConsoleExecutor prints to console,
 * 2) RcCommandUdpExecutor forwards data via UDP to alink_drone.
 */
class RcCommandConsoleExecutor : public IMspCommandExecutor {
public:
    void execute(const MspMessage& msg, FlightDataModel& dataModel) override {
        if (msg.size >= (16 * 2)) {
            memcpy(dataModel.channels, msg.payload, 16 * sizeof(uint16_t));
            if (dataModel.verbose) {
                cout << "[RcCommandConsoleExecutor] Channels:";
                for (int i = 0; i < CHANNEL_COUNT; i++) {
                    cout << " " << dataModel.channels[i];
                }
                cout << endl;
            }
        }
    }
};

class RcCommandAlinkForwarder : public IMspCommandExecutor {
public:
    explicit RcCommandAlinkForwarder(int outPort) {
        // Create a UDP socket for sending
        m_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock < 0) {
            perror("Failed to create output UDP socket");
            throw runtime_error("Output socket creation failed");
        }

        // Prepare destination address (localhost by default)
        memset(&m_destAddr, 0, sizeof(m_destAddr));
        m_destAddr.sin_family      = AF_INET;
        m_destAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        m_destAddr.sin_port        = htons(outPort);
    }

    ~RcCommandAlinkForwarder() override {
        if (m_sock >= 0) {
            close(m_sock);
        }
    }

    void execute(const MspMessage& msg, FlightDataModel& dataModel) override {
        if (msg.size >= (16 * 2)) {
            // channel 9 is the link quality;
            // channel 11: upper 5 bits - lost packets, lower 5 bits - recovered packets.
            // other channels are ignored.

            // output format:
            // TIMESTAMP:LINK_QUALITY:LINK_QUALITY:RECOVERED_PACKETS:LOST_PACKETS:20:20:20:20 (RSSI values are mocked, link quality is just duplicated)
            
            uint16_t link_quality = dataModel.channels[8];
            uint16_t lost_packets = dataModel.channels[10] & 0x1F;
            uint16_t recovered_packets = (dataModel.channels[11] >> 5) & 0x1F;

            char buffer[64];
            snprintf(buffer, sizeof(buffer), "%ld:%d:%d:%d:%d:20:20:20:20\n",
                     time(nullptr), link_quality, link_quality, recovered_packets, lost_packets);

            ssize_t sentBytes = sendto(
                m_sock,
                buffer,
                strlen(buffer),
                0,
                reinterpret_cast<sockaddr*>(&m_destAddr),
                sizeof(m_destAddr)
            );

            if (sentBytes < 0) {
                perror("Failed to send UDP data");
            }
        }
    }

private:
    int         m_sock { -1 };
    sockaddr_in m_destAddr {};
};

/******************************************************************************
 *                        MSP Command Dispatcher
 *
 * Dispatch an MspMessage to the list of executors  * registered for that
 * command. We can add multiple executors per command (chaining).
 ******************************************************************************/
class MspCommandDispatcher {
public:
    explicit MspCommandDispatcher(FlightDataModel& model)
        : m_dataModel(model)
    {
        // Register base executors
        registerExecutor(MspCommand::STATUS,     make_unique<StatusCommandExecutor>());
        registerExecutor(MspCommand::ATTITUDE,   make_unique<AttitudeCommandExecutor>());
        registerExecutor(MspCommand::FC_VARIANT, make_unique<FcVariantCommandExecutor>());
        // RC executors will be added externally (in main), demonstrating
        // Open/Closed for easy extension.
    }

    /**
     * Add an executor for a specific command.
     * We can add multiple executors for the same command.
     */
    void registerExecutor(MspCommand cmd, unique_ptr<IMspCommandExecutor> executor) {
        m_executors[cmd].push_back(move(executor));
    }

    /**
     * Dispatch the message to all executors registered for this command.
     */
    void dispatchMessage(const MspMessage& msg) {
        auto it = m_executors.find(msg.cmd);
        if (it != m_executors.end()) {
            for (auto& exec : it->second) {
                exec->execute(msg, m_dataModel);
            }
        } else if (m_dataModel.verbose) {
            cout << "[MspCommandDispatcher] Unhandled command: " 
                 << static_cast<int>(msg.cmd) << "\n";
        }
    }

private:
    FlightDataModel& m_dataModel;
    // Each command can have multiple executors
    map<MspCommand, vector< unique_ptr<IMspCommandExecutor> > > m_executors;
};

/******************************************************************************
 *                     MSP Message Handler Implementation
 *
 * Single Responsibility: Implementation of IMspMessageHandler. Receives fully
 * parsed messages and dispatches them to appropriate executors.
 ******************************************************************************/
class MspMessageHandler : public IMspMessageHandler {
public:
    explicit MspMessageHandler(FlightDataModel& model)
        : m_dataModel(model)
        , m_dispatcher(model)
    {
    }

    void onMspMessage(const MspMessage& msg) override {
        if (m_dataModel.verbose) {
            cout << "[MspMessageHandler] Received MSP msg: cmd="
                 << static_cast<int>(msg.cmd)
                 << ", size=" << static_cast<int>(msg.size) << "\n";
        }
        // Copy entire message to frame buffer if we want to forward it
        m_dataModel.copyToFrameBuffer(&msg, sizeof(msg));

        // Dispatch to all interested executors
        m_dispatcher.dispatchMessage(msg);
    }

    /**
     * Allow external configuration of the command dispatcher.
     * This is how we register new executors without changing the class.
     */
    MspCommandDispatcher& getDispatcher() {
        return m_dispatcher;
    }

private:
    FlightDataModel       m_dataModel;
    MspCommandDispatcher  m_dispatcher;
};

/******************************************************************************
 *                         Input Source Interfaces
 *
 * Single Responsibility: Each concrete input source (UDP, file) handles how
 * data is read, while the rest of the code simply calls receiveData().
 * 
 * For Liskov Substitution: UdpInputSource and FileInputSource must behave
 * consistently as IInputSource.
 ******************************************************************************/
class IInputSource {
public:
    virtual ~IInputSource() = default;
    virtual ssize_t receiveData(uint8_t* buffer, size_t bufferSize) = 0;
};

/**
 * UDP input source
 */
class UdpInputSource : public IInputSource {
public:
    explicit UdpInputSource(int port) {
        m_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_socket < 0) {
            perror("Failed to create UDP socket");
            throw runtime_error("Socket creation failed");
        }

        sockaddr_in serverAddr{};
        serverAddr.sin_family      = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port        = htons(port);

        if (bind(m_socket, reinterpret_cast<sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
            perror("Failed to bind UDP socket");
            close(m_socket);
            throw runtime_error("Socket binding failed");
        }

        cout << "[UdpInputSource] Listening on UDP port " << port << "...\n";
    }

    ~UdpInputSource() override {
        if (m_socket >= 0) {
            close(m_socket);
        }
    }

    ssize_t receiveData(uint8_t* buffer, size_t bufferSize) override {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);

        ssize_t bytesRead = recvfrom(
            m_socket,
            buffer,
            bufferSize,
            0,
            reinterpret_cast<sockaddr*>(&clientAddr),
            &clientLen
        );

        if (bytesRead < 0) {
            perror("Error receiving UDP data");
        }

        return bytesRead;
    }

private:
    int m_socket { -1 };
};

/**
 * File input source
 */
class FileInputSource : public IInputSource {
public:
    explicit FileInputSource(const string& filePath) {
        m_file.open(filePath, ios::binary);
        if (!m_file.is_open()) {
            throw runtime_error("Failed to open file: " + filePath);
        }
        cout << "[FileInputSource] Reading from file: " << filePath << "...\n";
    }

    ~FileInputSource() override {
        if (m_file.is_open()) {
            m_file.close();
        }
    }

    ssize_t receiveData(uint8_t* buffer, size_t bufferSize) override {
        if (!m_file.good()) {
            return -1; // signals EOF or error
        }
        m_file.read(reinterpret_cast<char*>(buffer), bufferSize);
        return m_file.gcount();
    }

private:
    ifstream m_file;
};

unique_ptr<IInputSource> createInputSource(string inputType, string source) {
    if (inputType == "udp") {
        int udpPort = stoi(source);
        if (udpPort <= 0 || udpPort > 65535) {
            throw invalid_argument("Invalid UDP port: " + source);
        }
        return make_unique<UdpInputSource>(udpPort);
    } 
    else if (inputType == "file") {
        return make_unique<FileInputSource>(source);
    } 
    else {
        throw invalid_argument("Invalid input type: " + inputType);
    }

    return nullptr;
}

/******************************************************************************
 *                               Main Function
 *
 * Orchestrates the wiring (composition root) of input sources, data model,
 * parser, handler, and executors.
 ******************************************************************************/
int main(int argc, char* argv[]) {
    // Usage: <exe> <input_type> <source> [out_udp_port]
    // e.g.   ./msp_parser udp 14555 9999
    //        (input from UDP port=14555, output alink commands to port=9999)
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input_type> <source> [out_udp_port]\n";
        cerr << "<input_type>: 'udp' or 'file'\n";
        cerr << "<source>: UDP port or file path\n";
        cerr << "[out_udp_port]: optional UDP port for RC output\n";
        return 1;
    }

    string inputType = argv[1];
    string source    = argv[2];

    // Optional outbound UDP port (for RC data)
    int outPort = 0;
    if (argc >= 4) {
        outPort = stoi(argv[3]);
        if (outPort <= 0 || outPort > 65535) {
            cerr << "Invalid outbound UDP port: " << argv[3] << "\n";
            return 1;
        }
    }

    try {
        // 1) Create input source
        unique_ptr<IInputSource> inputSource = createInputSource(inputType, source);

        // 2) Create the shared data model
        FlightDataModel flightModel;
        flightModel.verbose = true; // turn on console logs

        // 3) Create the top-level message handler (contains command dispatcher)
        MspMessageHandler messageHandler(flightModel);

        // 4) Register RC executors (chaining):
        //    a) Print to console
        auto consoleExec = make_unique<RcCommandConsoleExecutor>();
        messageHandler.getDispatcher().registerExecutor(MspCommand::RC, std::move(consoleExec));

        //    b) Send to alink if outPort was provided
        if (outPort > 0) {
            auto alinkExec = make_unique<RcCommandAlinkForwarder>(outPort);
            messageHandler.getDispatcher().registerExecutor(MspCommand::RC, std::move(alinkExec));
        }

        // 5) Create our parser, feeding it the message handler
        MspMessageParser parser(messageHandler);

        // 6) Read data in a loop, parse it byte-by-byte
        static const size_t BUFFER_SIZE = 1024;
        uint8_t buffer[BUFFER_SIZE] {};

        while (true) {
            ssize_t bytesRead = inputSource->receiveData(buffer, BUFFER_SIZE);
            if (bytesRead > 0) {
                for (ssize_t i = 0; i < bytesRead; ++i) {
                    parser.processByte(buffer[i]);
                }
            }
            else if (bytesRead == -1 && inputType == "file") {
                // End of file or error
                break;
            }
            // If in UDP mode and bytesRead == 0, we can just keep listening.
        }
    }
    catch (const exception& e) {
        cerr << "[ERROR] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
