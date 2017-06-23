#include <errno.h>
#include <signal.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <rcssnet/tcpsocket.hpp>
#include <rcssnet/exception.hpp>
#include <netinet/in.h>

#include "behaviours/behaviour.h"
#include "behaviours/naobehaviour.h"
#include "optimization/Optimizer.h"

#include "headers/headers.h"

using namespace rcss::net;


TCPSocket gSocket;
//UDPSocket gSocket;
std::string gHost = "127.0.0.1";
int gPort = 3100;

// Variables for connecting to the monitor port
TCPSocket mSocket;
std::string mHost = "127.0.0.1";
int mPort = -1;

unsigned long long key = 0;

// bool to indicate whether to continue the agent mainloop
static bool gLoop = true;

// Global variable of agent's body type to be exported
// (should probably make this not global and store this value in a better way)
int agentBodyType = 0;

// SIGINT handler prototype

extern "C" void handler(int sig) {
    if (sig == SIGINT)
        gLoop = false;
}

void PrintGreeting() {
    std::cout << "UTRA FC Base Code\n";

}

void PrintHelp() {
    std::cout << "\nusage: agentspark [options]" << std::endl;
    std::cout << "\noptions:" << std::endl;
    std::cout << " --help\tprints this message." << std::endl;
    std::cout << " --host=<IP>\tIP of the server." << std::endl;
    std::cout << " --port <port>\tport of the server." << std::endl;
    std::cout << " --type <type number>\theterogeneous model type number to use." << std::endl;
    std::cout << " --rsg <rsg>\trsg file for the nao model." << std::endl;
    std::cout << " --team <TeamName>\tName of Team." << std::endl;
    std::cout << " --unum <UNum>\tUniform Number of Player." << std::endl;
    std::cout << " --paramsfile <filename>\tname of a parameters file to be loaded" << std::endl;
    std::cout << " --pkgoalie\tgoalie for penalty kick shootout" << std::endl;
    std::cout << " --pkshooter\tshooter for penalty kick shootout" << std::endl;
    std::cout << " --optimize <agent-type>\toptimization agent type" << std::endl;
    std::cout << " --mhost=<IP>\tIP of the monitor for sending draw commands" << std::endl;
    std::cout << " --mport <port>\tport of the monitor for training command parser" << std::endl;

    std::cout << "\n";
}


/*
 * Read in parameters from inputsFile, which should be formatted
 * with a set of parameters as key value pairs from std::strings to
 * floats.  The parameter name should be separated from its value
 * with a tab and parameters should be separated from each other
 * with a single newline.  Parameters will be loaded into the
 * namedParams map.
 */
map<std::string, std::string> namedParams;

void LoadParams(const std::string& inputsFile) {
    std::istream *input;
    std::ifstream infile;
    std::istringstream inString;

    infile.open(inputsFile.c_str(), std::ifstream::in);

    if (!infile) {
        std::cerr << "Could not open parameter file " << inputsFile << std::endl;
        exit(1);
    }

    input = &(infile);

    std::string name;
    bool fBlockComment = false;
    while (!input->eof()) {

        // Skip comments and empty lines
        std::string str;
        std::getline(*input, str);
        if (str.length() >= 2 && str.substr(0, 2) == "/*") {
            fBlockComment = true;
        } else if (str == "*/") {
            fBlockComment = false;
        }
        if (fBlockComment || str == "" || str[0] == '#') {
            continue;
        }

        // otherwise parse std::strings
        std::stringstream s(str);
        std::string key;
        std::string value;
        std::getline(s, key, '\t'); //read thru tab
        std::getline(s, value); //read thru newline
        if (value.empty()) {
            continue;
        }
        namedParams[key] = value;
    }

    infile.close();
}


std::string teamName;
int uNum;
std::string outputFile(""); // For optimization
std::string agentType("naoagent");
std::string rsg("rsg/agent/nao/nao.rsg");

void ReadOptions(int argc, char* argv[]) {

    teamName = "UTRA_FC_Base";
    uNum = 0; // Value of 0 means choose next available number

    for (int i = 0; i < argc; i++) {

        if (strcmp(argv[i], "--help") == 0) {
            PrintHelp();
            exit(0);
        } else if (strncmp(argv[i], "--host", 6) == 0) {

            std::string tmp = argv[i];

            // minimal sanity check
            if (tmp.length() <= 7) {
                PrintHelp();
                exit(0);
            }

            gHost = tmp.substr(7);
        } else if (strncmp(argv[i], "--mhost", 7) == 0) {

            std::string tmp = argv[i];

            // minimal sanity check
            if (tmp.length() <= 8) {
                PrintHelp();
                exit(0);
            }

            mHost = tmp.substr(8);
        } else if (strncmp(argv[i], "--port", 6) == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            gPort = atoi(argv[i + 1]);
        } else if (strncmp(argv[i], "--mport", 7) == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            mPort = atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "--team") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            teamName = argv[i + 1];
        } else if (strcmp(argv[i], "--unum") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            uNum = atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "--paramsfile") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            std::string inputsFile = argv[i + 1];
            LoadParams(inputsFile);
        } else if (strcmp(argv[i], "--experimentout") == 0) {
            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }
            outputFile = argv[i + 1];
        } else if (strcmp(argv[i], "--optimize") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            agentType = argv[i + 1];
        } else if (strcmp(argv[i], "--type") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            rsg = "rsg/agent/nao/nao_hetero.rsg " + std::string(argv[i + 1]);
            agentBodyType = atoi(argv[i + 1]);
        } else if (strcmp(argv[i], "--rsg") == 0) {

            if (i == argc - 1) {
                PrintHelp();
                exit(0);
            }

            rsg = argv[i + 1];
        } else if (strcmp(argv[i], "--pkgoalie") == 0) {
            agentType = "pkgoalie";
        } else if (strcmp(argv[i], "--pkshooter") == 0) {
            agentType = "pkshooter";
        } else if (strcmp(argv[i], "--gazebo") == 0) {
            agentType = "gazebo";
        }

    } // for-loop
}

bool init() {
    std::cout << "connecting to TCP " << gHost << ":" << gPort << "\n";

    try {
        Addr local(INADDR_ANY, INADDR_ANY);
        gSocket.bind(local);
    } catch (BindErr error) {
        std::cerr << "failed to bind socket with '"
                << error.what() << "'" << std::endl;

        gSocket.close();
        return false;
    }

    try {
        Addr server(gPort, gHost);
        gSocket.connect(server);
    } catch (ConnectErr error) {

        std::cerr << "connection failed with: '"
                << error.what() << "'" << std::endl;
        gSocket.close();
        return false;
    }

    // Connect to the monitor port so that we can use the training command parser
    if (mPort != -1) {
        try {
            Addr local(INADDR_ANY, INADDR_ANY);
            mSocket.bind(local);
        } catch (BindErr error) {

            std::cerr << "failed to bind socket with '"
                    << error.what() << "'" << std::endl;

            mSocket.close();
            return false;
        }

        try {
            Addr server(mPort, gHost);
            mSocket.connect(server);
        } catch (ConnectErr error) {

            std::cerr << "connection failed with: '"
                    << error.what() << "'" << std::endl;
            mSocket.close();
            return false;
        }
    }

    return true;
}

void Done() {

    gSocket.close();
    std::cout << "closed connection to " << gHost << ":" << gPort << "\n";

    if (mPort != -1) {
        mSocket.close();
    }

}

bool SelectInput() {

	//why return 1 here? does the function work properly..?
    return 1;
    fd_set readfds;
    struct timeval tv = {60, 0};
    FD_ZERO(&readfds);
    FD_SET(gSocket.getFD(), &readfds);

    while (1) {
        switch (select(gSocket.getFD() + 1, &readfds, 0, 0, &tv)) {
            case 1:
                return 1;
            case 0:
                std::cerr << "(SelectInput) select failed " << strerror(errno) << std::endl;
                abort();
                return 0;
            default:
                if (errno == EINTR)
                    continue;
                std::cerr << "(SelectInput) select failed " << strerror(errno) << std::endl;
                abort();
                return 0;
        }
    }
}

void PutMessage(const std::string& msg) {
    //std::cout << msg << std::endl;

    if (msg.empty()) {
        return;
    }

    // prefix the message with it's payload length
    unsigned int len = htonl(msg.size());
    std::string prefix((const char*) &len, sizeof (unsigned int));
    std::string str = prefix + msg;

    if (static_cast<ssize_t> (str.size()) != write(gSocket.getFD(), str.data(), str.size())) {
        // LOG_STR("could not put entire message: " + msg);
    }
}

void PutMonMessage(const std::string& msg) {

    if (msg.empty()) {
        return;
    }

    // prefix the message with it's payload length
    unsigned int len = htonl(msg.size());
    std::string prefix((const char*) &len, sizeof (unsigned int));
    std::string str = prefix + msg;

    if (static_cast<ssize_t> (str.size()) != write(mSocket.getFD(), str.data(), str.size())) {
        // LOG_STR("could not put entire monitor message: " + msg);
    }
}

bool GetMessage(std::string& msg) {

    static char buffer[16 * 1024];

    unsigned int bytesRead = 0;
    while (bytesRead < sizeof (unsigned int)) {

        SelectInput();
        int readResult = read(gSocket.getFD(), buffer + bytesRead, sizeof (unsigned int) -bytesRead);

        if (readResult < 0) {
            continue;
        }

        if (readResult == 0) {
            // [patmac] Kill ourselves if we disconnect from the server
            // for instance when the server is killed.  This helps to
            // prevent runaway agents.
            std::cerr << "Lost connection to server" << std::endl;
            Done();
            exit(1);
        }

        bytesRead += readResult;
    }

    //std::cerr << "buffer = |" << std::string(buffer+1) << "|\n";
    //std::cerr << "bytesRead = |" << bytesRead << "|\n";
    //std::cerr << "Size of buffer = |" << sizeof(buffer) << "|\n";
    //std::cerr << "buffer = |" << buffer << "|\n";
    //std::cerr << "buffer[5] = |" << buffer[5] << "|\n";
    //printf ("xxx-%s\n", buffer+5);

    // msg is prefixed with it's total length

    union int_char_t {
        char *c;
        unsigned int *i;
    };

    int_char_t size;
    size.c = buffer;
    unsigned int msgLen = ntohl(*(size.i));
    // std::cerr << "GM 6 / " << msgLen << " (bytesRead " << bytesRead << ")\n";

    if (sizeof (unsigned int) +msgLen > sizeof (buffer)) {
        std::cerr << "too long message; aborting" << std::endl;
        abort();
    }

    // read remaining message segments
    unsigned int msgRead = bytesRead - sizeof (unsigned int);

    //std::cerr << "msgRead = |" << msgRead << "|\n";

    char *offset = buffer + bytesRead;

    while (msgRead < msgLen) {

        if (!SelectInput()) {
            return false;
        }

        unsigned readLen = sizeof (buffer) - msgRead;

        if (readLen > msgLen - msgRead) {
            readLen = msgLen - msgRead;
        }

        int readResult = read(gSocket.getFD(), offset, readLen);

        if (readResult < 0) {
            continue;
        }

        msgRead += readResult;
        offset += readResult;
        //std::cerr << "msgRead = |" << msgRead << "|\n";
    }

    // zero terminate received data
    (*offset) = 0;

    msg = std::string(buffer + sizeof (unsigned int));

    // DEBUG
    //std::cout << msg << std::endl;

    static std::string lastMsg = "";

    if (msg.compare(lastMsg) == 0) {
        std::cerr << "Duplicate message received from server -- has the server killed us?\n";
        Done();
        exit(1);
    }

    lastMsg = msg;

    return true;
}

void Run() {
    NaoBehaviour *behaviour;
    Optimizer *optimizer;

    if (agentType == "naoagent") {
        behaviour = new NaoBehaviour(teamName, uNum, namedParams, rsg);
        optimizer = new Optimizer(behaviour);
    } else {
        throw "unknown agent type";
    }
    std::cout << "Running!" << std::endl;

    PutMessage(behaviour->Init() + "(syn)");

    std::string msg;

    while (gLoop) {
        
        GetMessage(msg);
        std::string msgToServer = behaviour->Think(msg);

        //add fallen test
        //if(behaviour->isFallen()){
        //    cout << "what?I am fallen!" << endl;
        //}

        // To support agent sync mode
        msgToServer.append("(syn)");
        PutMessage(msgToServer);

        if (mPort != -1) {
            PutMonMessage(behaviour->getMonMessage());
        }
    }
}

int
main(int argc, char* argv[]) {
    // registering the handler, catching SIGINT signals
    signal(SIGINT, handler);


        ofstream file;
        file.open("test.txt");  
        file << "testing for opening the file" << endl;
        file.close();

        cout << "finishing" << endl;




    // Actually print out the errors that are thrown.
    try {
        PrintGreeting();
        ReadOptions(argc, argv);

        if (!init()) {
            return 1;
        }

        Run();
        Done();
    } catch (char const* c) {
        std::cerr << "-------------ERROR------------" << std::endl;
        std::cerr << c << std::endl;
        std::cerr << "-----------END ERROR----------" << std::endl;
    } catch (std::string s) {
        std::cerr << "-------------ERROR------------" << std::endl;
        std::cerr << s << std::endl;
        std::cerr << "-----------END ERROR----------" << std::endl;
    }
}
