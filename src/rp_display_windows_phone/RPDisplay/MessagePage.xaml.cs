/// <copyright file="MessagePage.xaml.cs">
///   Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
///   BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
/// </copyright>

﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Shapes;
using Microsoft.Phone.Controls;
using System.ComponentModel;
using System.Windows.Media.Imaging;
using ZXing;
using ZXing.Common;
using ZXing.QrCode;
using System.Threading;

namespace RPDisplay
{
    /// <summary>
    ///   Received message display page for the robot photographer's display app.
    /// </summary>
    public partial class MessagePage : PhoneApplicationPage
    {
        /// <summary>Robot's IP address.</summary>
        private string ipAddress;
        
        /// <summary>Robot's port.</summary>
        private string port;

        /// <summary>Client to retrieve messages from the robot photographer.</summary>
        private DisplayTCPClient displayTcpClient = new DisplayTCPClient();

        /// <summary>Background worker to poll for messages and render them in the UI.</summary>
        BackgroundWorker backgroundWorker = new BackgroundWorker();

        /// <summary>Default constructor.</summary>
        public MessagePage()
        {
            InitializeComponent();

            backgroundWorker.WorkerReportsProgress = false;
            backgroundWorker.WorkerSupportsCancellation = true;
            backgroundWorker.DoWork += new DoWorkEventHandler(PollForMessages);
        }

        /// <summary>Callback for background worker's polling of string messages from the robot.</summary>
        /// <param name="sender">Background worker's handle.</param>
        /// <param name="e">Background worker's callback arguments.</param>
        private void PollForMessages(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = sender as BackgroundWorker;

            while (!worker.CancellationPending)
            {
                // Receive a message from the robot over TCP and display it
                if (displayTcpClient.Connected)
                {
                    string robotsMessage = displayTcpClient.Receive();
                    Dispatcher.BeginInvoke(() => SetMessage(robotsMessage));
                }
                else
                {
                    ConnectToRobot(ipAddress, port);
                }
            }

            // Close the TCP connection on cancellation
            displayTcpClient.Close();
        }

        /// <summary>Generates a QR code from a given string message.</summary>
        /// <param name="message">Message which should be encoded into QR code.</param>
        /// <returns>Generated QR code bitmap.</returns>
        public WriteableBitmap GenerateQrCode(string message)
        {
            const int SIZE = 360;

            QRCodeWriter qrCodeWriter = new QRCodeWriter();
            BitMatrix qrCodeMatrix = qrCodeWriter.encode(message, BarcodeFormat.QR_CODE, SIZE, SIZE);

            WriteableBitmap result = new WriteableBitmap(SIZE, SIZE);
            for (int y = 0; y < SIZE; y++)
            {
                for (int x = 0; x < SIZE; x++)
                {
                    Color pixelColor = qrCodeMatrix[x, y] ? Colors.Black : Colors.White;
                    result.Pixels[(y * SIZE) + x] = pixelColor.A << 24 | pixelColor.R << 16 | pixelColor.G << 8 | pixelColor.B;
                }
            }

            return result;
        }

        /// <summary>
        ///   Fits the string message (and potentially a QR code) into the available
        ///   screen space.
        /// </summary>
        /// <param name="message">
        ///   Message which should be displayed (potentially containing a hyperlink).
        /// </param>
        /// <param name="justMessage">
        ///   Flag indicating whether only the text message, or the text message and
        ///   a QR code should be displayed.
        /// </param>
        private void FitMessage(string message, bool justMessage)
        {
            // Make the text block invisible
            textBlockMessage.Foreground = new SolidColorBrush(Colors.Transparent);

            // Collapse the QR code image
            imageQrCode.Visibility = System.Windows.Visibility.Collapsed;

            // Ensure that individual words fit horizontally without wrapping
            char[] separators = { ' ' };
            string[] messageWords = message.Split(separators);
            string splitMessage = messageWords.Aggregate((a, b) => a + "\n" + b);

            // Disable wrapping
            textBlockMessage.TextWrapping = TextWrapping.NoWrap;
            textBlockMessage.Text = splitMessage;

            int widthToFit = 450;
            while (textBlockMessage.ActualWidth <= widthToFit)
            {
                textBlockMessage.FontSize++;
                UpdateLayout();
            }

            while (textBlockMessage.ActualWidth > widthToFit)
            {
                textBlockMessage.FontSize--;
                UpdateLayout();
            }

            // Re-enable wrapping
            textBlockMessage.TextWrapping = TextWrapping.Wrap;
            textBlockMessage.Text = message;

            // Ensure that the message fits vertically
            double maxFontSize = textBlockMessage.FontSize;
            int heightToFit = justMessage ? 780 : 420;
            while ((textBlockMessage.ActualHeight <= heightToFit) && (textBlockMessage.FontSize <= maxFontSize))
            {
                textBlockMessage.FontSize++;
                UpdateLayout();
            }

            while ((textBlockMessage.ActualHeight > heightToFit) || (textBlockMessage.FontSize > maxFontSize))
            {
                textBlockMessage.FontSize--;
                UpdateLayout();
            }

            // Make the text block visible again
            textBlockMessage.Foreground = new SolidColorBrush(Colors.White);

            // Show the QR image
            if (!justMessage)
            {
                imageQrCode.Visibility = System.Windows.Visibility.Visible;
            }
        }

        /// <summary>Sets the message for rendering.</summary>
        /// <param name="message">
        ///   Message which should be rendered (potentially starting with a hyperlink, which will
        ///   be rendered as a QR code).
        /// </param>
        private void SetMessage(string message)
        {
            if (message.StartsWith("http://") || message.StartsWith("https://"))
            {
                int firstSpaceLocation = message.IndexOf(" ");
                if (-1 == firstSpaceLocation)
                {
                    firstSpaceLocation = message.Length - 1;
                }

                string url = message.Substring(0, firstSpaceLocation);
                string text = message.Substring(firstSpaceLocation + 1);

                FitMessage(text, false);
                imageQrCode.Source = GenerateQrCode(url);
            }
            else
            {
                FitMessage(message, true);
            }
        }

        /// <summary>Opens a TCP connection to the robot.</summary>
        /// <param name="ipAddress">Robot's IP address.</param>
        /// <param name="port">Robot's port.</param>
        private void ConnectToRobot(string ipAddress, string port)
        {
            // Connect to the robot
            while (!displayTcpClient.Connect(ipAddress, Int32.Parse(port)))
            {
                Dispatcher.BeginInvoke(() => SetMessage(String.Format("Cannot reach robot at {0}:{1}. Retrying...", ipAddress, port)));
                Thread.Sleep(1000);
            }

            Dispatcher.BeginInvoke(() => SetMessage("Connected to robot!"));
        }

        /// <summary>
        ///   Navigation to page handler, which opens a TCP connection to the robot and starts
        ///   the background worker which polls the messages from the robot.
        /// </summary>
        /// <param name="e">
        ///   Navigation arguments, containing an IP address (under the key "ip") and the port
        ///   (under the key "port").
        /// </param>
        protected override void OnNavigatedTo(System.Windows.Navigation.NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);

            // Save robot's connection parameters
            ipAddress = NavigationContext.QueryString["ip"];
            port = NavigationContext.QueryString["port"];

            // Start the message update background worker
            backgroundWorker.RunWorkerAsync();
        }
    }
}