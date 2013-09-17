/// <copyright file="DisplayTCPClient.cs">
///   Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
///   BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
/// </copyright>

﻿using System;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using System.Net.Sockets;
using System.Threading;
using System.Text;

namespace RPDisplay
{
    /// <summary>
    ///   Synchronous TCP client which connects to a given server and returns received
    ///   string messages.
    /// </summary>
    public class DisplayTCPClient
    {
        /// <summary>TCP socket.</summary>
        private Socket socket;

        /// <summary>Signal object.</summary>
        static ManualResetEvent clientDone = new ManualResetEvent(false);

        /// <summary>TCP buffer size constant.</summary>
        const int BUFFER_SIZE = 65536; // 64 kB
        
        /// <summary>TCP connection timeout constant (ms).</summary>
        const int CONNECTION_TIMEOUT_MS = 30000; // 30 seconds

        /// <summary>Default constructor.</summary>
        public DisplayTCPClient()
        {
            // Create the TCP socket
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        }

        /// <summary>Opens a TCP connection to a given server.</summary>
        /// <param name="ipAddress">Server's IP address.</param>
        /// <param name="port">Server's port.</param>
        /// <returns>Connection's success status.</returns>
        public bool Connect(string ipAddress, int port)
        {
            // Set up the asynchronous event arguments
            SocketAsyncEventArgs socketEventArgs = new SocketAsyncEventArgs();
            socketEventArgs.RemoteEndPoint = new IPEndPoint(IPAddress.Parse(ipAddress), port);
            socketEventArgs.Completed += new EventHandler<SocketAsyncEventArgs>((object o, SocketAsyncEventArgs e) =>
            {
                // Unblock when completed
                clientDone.Set();
            });

            // Clear the event to block the thread until connection
            clientDone.Reset();

            // Connect synchronously
            socket.ConnectAsync(socketEventArgs);
            clientDone.WaitOne(CONNECTION_TIMEOUT_MS);

            return socket.Connected;
        }

        /// <summary>Synchronously receives a string from the server.</summary>
        /// <returns>Received string.</returns>
        public string Receive()
        {
            string result = "Timed out waiting for robot photographer's response.";

            // Check if socket was created
            if (socket != null)
            {
                // Create asynchronous event arguments
                SocketAsyncEventArgs socketEventArgs = new SocketAsyncEventArgs();
                socketEventArgs.RemoteEndPoint = socket.RemoteEndPoint;
                socketEventArgs.SetBuffer(new Byte[BUFFER_SIZE], 0, BUFFER_SIZE);
                socketEventArgs.Completed += new EventHandler<SocketAsyncEventArgs>((object o, SocketAsyncEventArgs e) =>
                {
                    // If received data
                    if (SocketError.Success == e.SocketError)
                    {
                        result = Encoding.UTF8.GetString(e.Buffer, e.Offset, e.BytesTransferred);
                        result = result.Trim('\0');
                    }
                    else
                    {
                        // Return error message
                        result = e.SocketError.ToString();
                    }

                    // Unblock when completed
                    clientDone.Set();
                });

                // Block the thread until completion of an asynchronous request
                clientDone.Reset();

                // Make the request
                socket.ReceiveAsync(socketEventArgs);
                clientDone.WaitOne();
            }
            else
            {
                result = "Not connected to robot photographer.";
            }

            return result;
        }

        /// <summary>Closes the TCP connection.</summary>
        public void Close()
        {
            if (socket != null)
            {
                socket.Close();
            }
        }

    }
}