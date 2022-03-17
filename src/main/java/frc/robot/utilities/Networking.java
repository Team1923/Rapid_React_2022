// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;

/* This class was made with notes from:

https://www.chiefdelphi.com/t/best-frc-programming-features/353571/36

and piecing together from this:
http://www.java2s.com/Tutorials/Java/Network/IP/Get_IP_address_from_NetworkInterface_in_Java.htm
*/

// WARNING: This assumes your roboRIO has a static IP set on eth0, which it should *anyways*.
public class Networking {
  public static double teamNumber() {
    ArrayList<String> ips = new ArrayList<String>();

    try {
      NetworkInterface neti = NetworkInterface.getByName("eth0");
      Enumeration<InetAddress> addresses = neti.getInetAddresses();
      // a network interface can have multiple addresses, even if ours doesn't, so we traverse all
      // of them.

      while (addresses.hasMoreElements()) {
        InetAddress addr = addresses.nextElement();
        if (addr instanceof Inet4Address && !addr.isLoopbackAddress()) {
          ips.add(addr.getHostAddress());
        }
      }
    } catch (Exception e) {
    }

    for (String ip : ips) {
      String[] ipOctets = ip.split(".");
      if (ipOctets[0] == "10") { // if a FMS static IP range.
        return Double.parseDouble(ipOctets[1] + ipOctets[2]);
      }
    }
    return 0;
  }
}
