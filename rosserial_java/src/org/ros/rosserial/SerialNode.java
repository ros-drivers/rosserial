package org.ros.rosserial;
import org.ros.rosserial.*;


import gnu.io.SerialPort;

import org.ros.Node;
import org.ros.NodeConfiguration;
import org.ros.NodeMain;
import org.ros.Publisher;

import com.google.common.base.Preconditions;

import java.io.FileOutputStream;

public class SerialNode implements NodeMain {

	  private Node node;


	public static SerialPort createSerialPort(String portName){
		Enumeration portIdentifiers = CommPortIdentifier.getPortIdentifiers();
		
		CommPortIdentifier portId = null;  // will be set if port found
		
		while (portIdentifiers.hasMoreElements())
		{
		    CommPortIdentifier pid = (CommPortIdentifier) portIdentifiers.nextElement();
		    if(pid.getPortType() == CommPortIdentifier.PORT_SERIAL &&
		       pid.getName().equals(portName)) 
		    {
		        portId = pid;
		        break;
		    }
		}
		if(portId == null)
		{
		    System.err.println("Could not find serial port " + portName);
		    System.exit(1);
		}
		
		SerialPort port = null;
		try {
		    port = (SerialPort) portId.open(
		        "name", // Name of the application asking for the port 
		        10000   // Wait max. 10 sec. to acquire port
		    );
		} catch(PortInUseException e) {
		    System.err.println("Port already in use: " + e);
		    System.exit(1);
		}
	
		return port;
	}

	  @Override
	  public void main(NodeConfiguration configuration) {
	    Preconditions.checkState(node == null);
	    Preconditions.checkNotNull(configuration);
	    try {
		  System.out.println("Starting RosSerial node");
	      node = new Node("rosserial_node", configuration);

			String portName = "/dev/ttyUSB0";
			SerialPort port= createSerialPort();
		
			port.setSerialPortParams(57600,    
					SerialPort.DATABITS_8,
					SerialPort.STOPBITS_1,
					SerialPort.PARITY_NONE);
					
					
			//FileOutputStream out = port.getOutputStream();//new FileOutputStream("rjs_out.txt");
			
			RosSerial rs = new RosSerial(node, port.getInputStream(), port.getOutputStream());
			System.out.println("Now running RosSerial Node connected to " + portName);
			rs.run();	
		}catch (Exception e) {
	      if (node != null) {
		        e.printStackTrace();

	        node.getLog().fatal(e);
	      } else {
	        e.printStackTrace();
	      }
	    }
	  }

	  @Override
	  public void shutdown() {
	    node.shutdown();
	    node = null;
	  }

	}
