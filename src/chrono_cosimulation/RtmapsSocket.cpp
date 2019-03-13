// =============================================================================
// Authors: Rudloff Valentin
// Used to read and send data over TCP/IP with RtMaps
// =============================================================================

#include <vector>

#include "chrono_cosimulation/RtmapsSocket.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

using namespace chrono::cosimul;

namespace chrono{
namespace cosimul{

RtmapsSocket::RtmapsSocket(ChSocketFramework& mframework,
                               int n_in_values,  /// number of scalar variables to receive each timestep
                               int n_out_values  /// number of scalar variables to send each timestep
                               ) {
    this->myServer = 0;
    this->myClient = 0;
    this->in_n = n_in_values*15;
    this->out_n = n_out_values;
    this->nport = 0;
}

RtmapsSocket::~RtmapsSocket() {
    if (this->myServer)
        delete this->myServer;
    this->myServer = 0;
    if (this->myClient)
        delete this->myClient;
    this->myClient = 0;
}

bool RtmapsSocket::WaitConnection(int aport) {
    this->nport = aport;

    // a server is created, that could listen at a given port
    this->myServer = new ChSocketTCP(aport);

    // bind socket to server
    this->myServer->bindSocket();

    // wait for a client to connect (this might put the program in
    // a long waiting state... a timeout can be useful then)
    this->myServer->listenToClient(1);

    std::string clientHostName;
    this->myClient = this->myServer->acceptClient(clientHostName);  // pick up the call!

    if (!this->myClient)
        throw(ChExceptionSocket(0, "Server failed in getting the client socket"));

    return true;
}

bool RtmapsSocket::SendData(double mtime, ChMatrix<double>* out_data) {
    if (out_data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with 1 column");
    if (out_data->GetRows() != this->out_n)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'SendData' with no connected client.");

    std::vector<char> mbuffer;                     // now zero length
    ChStreamOutBinaryVector stream_out(&mbuffer);  // wrap the buffer, for easy formatting

    // Serialize datas (little endian)...

    // time:
    stream_out << mtime;
    // variables:
    for (int i = 0; i < out_data->GetRows(); i++)
        stream_out << out_data->Element(i, 0);

    // -----> SEND!!!
    this->myClient->SendBuffer(*stream_out.GetVector());

    return true;
}

bool RtmapsSocket::ReceiveData(double& mtime, ChMatrix<double>* in_data) {
    if (in_data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with 1 column");
    if (in_data->GetRows() != this->in_n)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'ReceiveData' with no connected client.");

    // Receive from the client
    int nbytes = sizeof(double) * (this->in_n + 1);
    std::vector<char> rbuffer;
    rbuffer.resize(nbytes);                      // reserve to number of expected bytes
    ChStreamInBinaryVector stream_in(&rbuffer);  // wrap the buffer, for easy formatting

    // -----> RECEIVE!!!
    int numBytes = this->myClient->ReceiveBuffer(*stream_in.GetVector(), nbytes);

    // Deserialize datas (little endian)...

    // time:
    stream_in >> mtime;
    // variables:
    for (int i = 0; i < in_data->GetRows(); i++)
        stream_in >> in_data->Element(i, 0);

    return true;
}

bool RtmapsSocket::SendData(int mtime, ChMatrix<double>* out_data)
{
    if (out_data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with 1 column");
    if (out_data->GetRows() != this->out_n)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'SendData' with no connected client.");

    std::vector<char> mbuffer;                     // now zero length
    ChStreamOutBinaryVector stream_out(&mbuffer);  // wrap the buffer, for easy formatting

    // Serialize datas (little endian)...

    // time:
    stream_out << mtime;
    // variables:
    for (int i = 0; i < out_data->GetRows(); i++)
        stream_out << out_data->Element(i, 0);

    // -----> SEND!!!
    this->myClient->SendBuffer(*stream_out.GetVector());

    return true;
}

bool RtmapsSocket::ReceiveData(int& mtime, ChMatrix<double>* in_data)
{
    if (in_data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with 1 column");
    if (in_data->GetRows() != this->in_n)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'ReceiveData' with no connected client.");

    // Receive from the client
    int nbytes = sizeof(double) * (this->in_n + 1);
    std::vector<char> rbuffer;
    rbuffer.resize(nbytes);                      // reserve to number of expected bytes
    ChStreamInBinaryVector stream_in(&rbuffer);  // wrap the buffer, for easy formatting

    // -----> RECEIVE!!!
    int numBytes = this->myClient->ReceiveBuffer(*stream_in.GetVector(), nbytes);

    // Deserialize datas (little endian)...

    // time:
    stream_in >> mtime;
    // variables:
    for (int i = 0; i < in_data->GetRows(); i++)
        stream_in >> in_data->Element(i, 0);

    return true;
}

bool RtmapsSocket::SendData(int mtime, ChMatrix<int>* out_data)
{
    if (out_data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with 1 column");
    if (out_data->GetRows() != this->out_n)
        throw ChExceptionSocket(0, "Error. Sent data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'SendData' with no connected client.");

    std::vector<char> mbuffer;                     // now zero length
    ChStreamOutBinaryVector stream_out(&mbuffer);  // wrap the buffer, for easy formatting

    // Serialize datas (little endian)...

    // time:
    stream_out << mtime;
    // variables:
    for (int i = 0; i < out_data->GetRows(); i++)
        stream_out << out_data->Element(i, 0);

    // -----> SEND!!!
    this->myClient->SendBuffer(*stream_out.GetVector());

    return true;
}

bool RtmapsSocket::ReceiveData(int& mtime, std::vector<int>& in_data)
{
    ChMatrix<int>* data;
    if (data->GetColumns() != 1)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with 1 column");
    if (data->GetRows() != this->in_n)
        throw ChExceptionSocket(0, "Error. Received data must be a matrix with N rows and 1 column");
    if (!myClient)
        throw ChExceptionSocket(0, "Error. Attempted 'ReceiveData' with no connected client.");

    // Receive from the client
    int nbytes = sizeof(double) * (this->in_n + 1);
    std::vector<char> rbuffer;
    rbuffer.resize(nbytes);                      // reserve to number of expected bytes
    ChStreamInBinaryVector stream_in(&rbuffer);  // wrap the buffer, for easy formatting

    // -----> RECEIVE!!!
    int numBytes = this->myClient->ReceiveBuffer(*stream_in.GetVector(), nbytes);

    // Deserialize datas (little endian)...

    // time:
    stream_in >> mtime;
    // variables:
    for (int i = 0; i < data->GetRows(); i++)
        stream_in >> data->Element(i, 0);

    int iteration = 0;
    bool next_data = false;
    do
    {
        in_data[iteration * 3] = data->GetElement(9 + 15 * iteration, 0);
        in_data[iteration * 3 + 1] = data->GetElement(6 + 15 * iteration, 0);
        in_data[iteration * 3 + 2] = data->GetElement(13 + 15 * iteration, 0);
        next_data = (data->GetElement(14 + 15*iteration, 0) != 0 || iteration >= in_data.size()/3);
        iteration++;
    } while (next_data);


    return true;
}

}  // end namespace cosimul
}  // end namespace chrono
