/// <copyright file="MainPage.xaml.cs">
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
using System.Net.Sockets;

namespace RPDisplay
{
    /// <summary>
    ///   Connection settings page for the robot photographer's display app.
    /// </summary>
    public partial class MainPage : PhoneApplicationPage
    {
        /// <summary>Default constructor.</summary>
        public MainPage()
        {
            InitializeComponent();
        }

		/// <summary>Connect button click handler.</summary>
        /// <param name="sender">Sender object.</param>
        /// <param name="e">Button click arguments.</param>
        private void ApplicationBarIconButton_Click(object sender, EventArgs e)
        {
            string uri = string.Format("/MessagePage.xaml?ip={0}&port={1}", textBoxIpAddress.Text, textBoxPort.Text);

            NavigationService.Navigate(new Uri(uri, UriKind.Relative));
        }
    }
}