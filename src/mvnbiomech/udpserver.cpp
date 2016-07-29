/*! \file
	\section FileCopyright Copyright Notice
	This is free and unencumbered software released into the public domain.

	Anyone is free to copy, modify, publish, use, compile, sell, or
	distribute this software, either in source code form or as a compiled
	binary, for any purpose, commercial or non-commercial, and by any
	means.

	In jurisdictions that recognize copyright laws, the author or authors
	of this software dedicate any and all copyright interest in the
	software to the public domain. We make this dedication for the benefit
	of the public at large and to the detriment of our heirs and
	successors. We intend this dedication to be an overt act of
	relinquishment in perpetuity of all present and future rights to this
	software under copyright law.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
	MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
	IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
	OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
	ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
	OTHER DEALINGS IN THE SOFTWARE.
*/

#include "udpserver.h"

UdpServer::UdpServer(std::string address, uint16_t port) : Server(address, port)
{
  m_port = port;
  m_hostName = address;

  m_parserManager.reset(new ParserManager(true, false));
  m_socket.reset(new Socket(IP_UDP));

  bool res = m_socket->bind(m_port);
  if (res == true)
    startRead();
}

UdpServer::~UdpServer()
{
}

void UdpServer::startRead()
{
  char buffer[MAX_MVN_DATAGRAM_SIZE]; // Max datagram size 1588

  //std::cout << "Waiting to receive packets from the client ..." << std::endl << std::endl;

  while(true) 
  {
    int rv = m_socket->read(buffer, MAX_MVN_DATAGRAM_SIZE);
    if (rv > 0)
    {
    m_parserManager->readDatagram(buffer);
    }
  
    //buffer.clear();
    std::memset(buffer, 0, MAX_MVN_DATAGRAM_SIZE);
  }
}
