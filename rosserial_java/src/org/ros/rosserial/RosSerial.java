package org.ros.rosserial;
import org.ros.rosserial.*;


import java.io.*;
import java.security.KeyException;
import java.util.*;
import gnu.io.*;


import org.ros.exception.RosInitException;
import org.ros.message.*;
import org.ros.*;

import org.ros.MessageListener;
import org.ros.message.Message;



public class RosSerial implements Runnable{
	//define some modes for the serial communication state machine	
	private Boolean  running  = false;
	
	static Boolean debug = false;
	
	Node node;
	
	OutputStream ostream;
	BufferedInputStream istream;
	

	Hashtable<Integer, Publisher> publishers = new Hashtable();
	Hashtable<Integer, Subscriber> subscriber = new Hashtable();
	
	
	Hashtable<String, Integer  > id_lookup= new Hashtable();
	Hashtable<Integer, String  > topic_lookup = new Hashtable();
	Hashtable<Integer, Class > msg_classes = new Hashtable();

	
	public RosJSerial(Node nh, InputStream input, OutputStream output){
		node = nh;
		ostream = output;
		istream = new BufferedInputStream(input,100);
		}
	
	
	
	private  Class loadMsgClass(String msg) throws ClassNotFoundException{
		String[] msgParts = msg.split("/");
		Class aClass = RosJSerial.class.getClassLoader().loadClass("org.ros.message."+msgParts[0]+"."+msgParts[1]);
		System.out.println("Loading Msg Class : " + aClass.getName());
		return aClass;
	}
	
	private void requestTopics(){
		byte[] request= {(byte) 0x0FF, (byte) 0x0FF, (byte) 0, (byte) 0,(byte) 0, (byte)0x10 };
		byte[] flushing = new byte[50];
		for(int i =0; i<50; i++) flushing[i]=0;
		try{
			ostream.write(flushing);
			Thread.sleep(200);
			ostream.write(request);
			ostream.flush();
			Thread.sleep(200);

			ostream.write(flushing);
			Thread.sleep(200);
			ostream.write(request);
			ostream.flush();
			Thread.sleep(200);

		}
		catch(IOException e){
			System.out.println("Failed sending topic request : " +e.toString());
		}
		catch(java.lang.InterruptedException e){
		e.printStackTrace();
		}
	}
	
	private boolean addTopic(String topic, String topic_type, int id){
		System.out.println("Adding topic " + topic + " of type " + topic_type +" : " + id);
		
		try{
			Class msg_class = loadMsgClass(topic_type);
			msg_classes.put(id, msg_class);
			
			id_lookup.put(topic, id);
			topic_lookup.put(id, topic);
			
			if (id <128){
				Publisher pub = node.createPublisher(topic, msg_class);
				publishers.put(id,pub);
			}
			else{
				Subscriber sub = node.createSubscriber(topic, new MessageListenerRosSerial(this, id, 0), msg_class );
			}	
			return true;
		}
		catch (Exception e){
			e.printStackTrace();
			}
		return false;
	}
	 
	
	/*
	 * This is a bootleg read method because I was having trouble with the serial port strangely returning.
	 */
	private void read(byte[] buff){
		int i=0;
		try{
			while (i<buff.length)  i+= istream.read(buff, i, buff.length-i); 
		}
		catch (Exception e){
			e.printStackTrace();
		}
	}
	
	public void run(){


	    running = true;

		requestTopics();
	    System.out.println("Topics requested");

	    
	    byte[] flags = new byte [2];
	 
	    
		while (running){
			try{
				read(flags);
				if ( !(flags[1] == (byte) 0xff && 
					  flags[ 0] == (byte) 0xff) ) {
					continue;
				}



				int topic_id = istream.read();
				if (topic_id == 0){ // receiving topic update
					
					int data_l = istream.read();
					
					int n_id = istream.read(); //new topic id
					
					int topic_name_len = istream.read();
				if (debug)	System.out.println("Topic name length is " + topic_name_len);
					byte[] topic_name_bytes = new byte[topic_name_len];
					read(topic_name_bytes );
					String topic_name = new String(topic_name_bytes);
					
					int topic_type_len = istream.read();
					byte[] topic_type_bytes = new byte[topic_type_len];
					if (debug) System.out.println("Topic typelength is " + topic_type_len);

					read(topic_type_bytes);
					String topic_type = new String(topic_type_bytes);
					
					addTopic(topic_name, topic_type, n_id);
					if (debug) System.out.println("There are now " +  msg_classes.size() + " keys ");
				}
				else{
					int chk =1+(byte) topic_id;
					if (debug)	System.out.println("Topic id is " + topic_id);
					
					
					int topic_type = istream.read();
					chk += topic_type;
					
					byte[] l_data_b = new byte[2];
					read(l_data_b);
					chk += l_data_b[0]; chk+= l_data_b[1];
					int l_data = (int)(l_data_b[0] <<  8) | (int)(l_data_b[1]);
					if (debug)	System.out.println("The data length is "+ l_data);
					
					
					byte[] data = new byte[l_data]; //subtract one because the checksum isnt included...
					read(data);
					for(int i =0; i <data.length; i++) chk += data[i];
					if (debug)	 System.out.println("The checksum is " + chk);
					
					if (chk%256 == 0){
						try{
      						Message msg = (Message) msg_classes.get(topic_id).newInstance();
							//System.out.println("About to deserialize");
							msg.deserialize(data);
							//System.out.println("Deserialize msg");
							publishers.get(topic_id).publish(msg);
						}
						catch (Exception e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
					else{
						System.out.println("Checksum failed");
					}
				}
			}
			catch(IOException e ){
				e.printStackTrace();
			}
		}

	}
	
	public void send(int id, int type, org.ros.message.Message t){
		
		
		int l = t.serializationLength();
		
		byte[] packet_header = new byte[2+1+1+2]; //sync_flags + topic_id + data_len + data + checksum
		packet_header[0] = (byte) 0xff;
		packet_header[1] = (byte) 0xff;
		packet_header[2] = (byte) id;
		packet_header[3] = (byte) type;
		packet_header[4] = (byte) (l >> 8);
		packet_header[5] = (byte) l;
		
		byte[] data = t.serialize(0);
		
		int chk = 0;
		for (int i=2; i<6; i++) chk+= packet_header[i];
		for(int i=0;  i<l; i++) chk+= data[i];
		chk = 255-chk%256;
		try{
			ostream.write(packet_header);
			ostream.write(data);
			ostream.write(chk);
		}catch(IOException e)
		{
			e.printStackTrace();
		}
	}
	
}
