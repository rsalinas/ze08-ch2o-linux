/**
 * Written by Raúl Salinas-Monteagudo <rausalinas@gmail.com>
 * 2018-11-25
 *
 * This Linux program reads data from a Winsen ze08-ch20 module.
 *
 * Those modules come with a serial interface, and you can simply
 * connect 5V/GND and TX to get the values every second.
 *
 * This program accepts a commit interval as a parameter.  This can
 * reduce the disk wearout.
 */

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <csignal>
#include <vector>

using namespace std;

struct Ze08Ch20 {
    typedef unsigned short concentration_t;
    struct Measurement {
        time_t ts;
        concentration_t concentration;
    };
    std::vector<Measurement> buffer_;
    Ze08Ch20(const std::string& port, size_t interval)
        : fd_(open(port.c_str(), O_RDWR| O_NOCTTY))
        , commitInterval(interval) {
        if (fd_ < 0) {
            perror("Cannot open serial port");
            exit(1);
        }
        struct termios tty;

        memset(&tty, 0, sizeof tty);

        if ( tcgetattr ( fd_, &tty ) != 0 ) {
            std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        }

        /* Save old tty parameters */
        tty_old = tty;

        cfsetospeed (&tty, speed_t(B9600));
        cfsetispeed (&tty, speed_t(B9600));

        tty.c_cflag     &=  tcflag_t(~PARENB)            // Make 8n1
                &  tcflag_t(~CSTOPB)
                &  tcflag_t(~CSIZE);
        tty.c_cflag     |=  CS8;
        tty.c_cflag     &=  tcflag_t(~CRTSCTS)            // no flow control
                & tcflag_t(~ICANON); /* Set non-canonical mode */
        tty.c_cc[VMIN]   = 1; // 1;                  // read doesn't block
        tty.c_cc[VTIME]  = 5; // 5;                  // 0.5 seconds read timeout
        tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

        cfmakeraw(&tty);

        tcflush(fd_, TCIFLUSH );
        if ( tcsetattr ( fd_, TCSANOW, &tty ) != 0) {
            std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        }
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);

        buffer_.reserve(commitInterval);
        clog << "Memory used for buffering: " << sizeof (Measurement) * commitInterval << endl;
    }
    static void signalHandler(int signo) {
        clog << "Signal: " << strsignal(signo) << endl;
        if (running_) {
            running_ = false;
        } else {
            exit(1);
        }
    }
    struct termios tty_old;
    ~Ze08Ch20() {
        if (buffer_.size()) {
            clog << "Pending data: " << endl;
            flushBuffer();
        }
        tcsetattr(fd_, 0, &tty_old);
    }
    enum State : char {
        STATE_START, STATE_GASNAME, STATE_UNIT, STATE_NODECIMAL,
        STATE_CONCENTRATION_HI, STATE_CONCENTRATION_LO,
        STATE_FULLRANGE_HI, STATE_FULLRANGE_LO, STATE_CHECKSUM
    } state = STATE_START;
    static volatile bool running_;
    void resetState() {
        state = STATE_START;
    }
    void flushBuffer() {
        for (const auto& m : buffer_) {
            clog << "Data: " << m.ts << " " << m.concentration << endl;
        }
        buffer_.clear();
    }


    void processMeasurement(concentration_t concentration, concentration_t /* unused fullrange*/) {
        auto now = time(nullptr);
        buffer_.emplace_back(Measurement{now, concentration});
        if (buffer_.size() == commitInterval) {
            flushBuffer();
        }
    }
    bool run() {
        unsigned char dataSum = 0;
        concentration_t concentration = 0, fullrange = 0;
        concentration_t lastConcentration = 0;
        while (running_) {
            unsigned char ch;
            auto bytes = read(fd_, &ch, sizeof ch);
            switch (bytes) {
            case -1: {
                auto savedErrno = errno;
                clog << "Port disappeared: " << strerror(savedErrno) << endl;
                return false;
            }
            case 0:
                clog << "EOF" << endl;
                return false;
            default:
                switch (state) {
                case STATE_START:
                    if (ch == 0xFFu) {
                        dataSum = 0;
                        state = STATE_GASNAME;
                        continue;
                    }
                    break;
                case STATE_GASNAME:
                    if (ch != 0x17) {
                        resetState();
                        continue;
                    }
                    break;
                case STATE_UNIT:
                    if (ch != 0x04) {
                        resetState();
                        continue;
                    }
                    break;
                case STATE_NODECIMAL:
                    if (ch != 0) {
                        resetState();
                        continue;
                    }
                    break;
                case STATE_CONCENTRATION_HI:
                    concentration = static_cast<unsigned short>(ch << 8);
                    break;
                case STATE_CONCENTRATION_LO:
                    concentration |= ch;
                    break;
                case STATE_FULLRANGE_HI:
                    fullrange = static_cast<unsigned short>(ch << 8);
                    break;
                case STATE_FULLRANGE_LO:
                    fullrange |= ch;
                    break;
                case STATE_CHECKSUM:
                    unsigned char finalEcksum  = (~dataSum)+1 ;
                    if (finalEcksum   == ch ) {
                        if (lastConcentration != concentration) {
                            processMeasurement(concentration, fullrange);
                            lastConcentration = concentration;
                        }
                    } else {
                        clog << "Wrong checksum: " << std::hex << +finalEcksum
                             <<" vs " << +ch << std::dec << endl;
                    }
                    resetState();
                    continue;

                }
                dataSum += ch;
                state = State(state + 1);
            }
        }
        return true;
    }

private:
    int fd_;
    const size_t commitInterval;
};

volatile bool Ze08Ch20::running_ = true;

int main(int argc, const char * const *argv)
{
    if (argc == 1) {
        clog << "Usage: " << argv[0] << " <port> [commit interval (defaults to 1)]" << endl;
        return EXIT_FAILURE;
    }
    return Ze08Ch20{argv[1], argc >= 3 ? std::stoul(argv[2]) : 1L}.run() ? EXIT_SUCCESS : EXIT_FAILURE;
    }
