// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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

import org.ros.message.rosserial_msgs.*;
import org.ros.message.std_msgs.Time;


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

	static final int TOPIC_PUBLISHERS = 0;
	static final int TOPIC_SUBSCRIBERS = 1;
	static final int TOPIC_TIME = 10;

	
	public RosSerial(Node nh, InputStream input, OutputStream output){
		node = nh;
		ostream = output;
		istream = new BufferedInputStream(input,100);
		}
	
	private  Class loadMsgClass(String msg) throws ClassNotFoundException{
		String[] msgParts = msg.split("/");
		Class aClass = RosSerial.class.getClassLoader().loadClass("org.ros.message."+msgParts[0]+"."+msgParts[1]);
		System.out.println("Loading Msg Class : " + aClass.getName());
		return aClass;
	}
	
	private void requestTopics(){
		byte[] request= {(byte) 0x0FF, (byte) 0x0FF, (byte) 0, (byte) 0,(byte) 0, (byte) 0, (byte)0xff };
		byte[] flushing = new byte[50];
		for(int i =0; i<50; i++) flushing[i]=0;
		try{
			ostream.write(flushing);
			Thread.sleep(200);
			ostream.write(request);
			ostream.flush();
			Thread.sleep(500);

			ostream.write(flushing);
			Thread.sleep(200);
			ostream.write(request);
			ostream.flush();
			Thread.sleep(200);

		}
		catch(Exception e){
		System.out.println("Failed sending topic request : " +e.toString());
		e.printStackTrace();
		}
	}
	
	private boolean addTopic(String topic, String topic_type, int id, boolean Publisher){
		if ( topic.equals(topic_lookup.get(id)) ) return true;
		System.out.println("Adding topic " + topic + " of type " + topic_type +" : " + id);

		try{
			Class msg_class = loadMsgClass(topic_type);
			msg_classes.put(id, msg_class);
			
			id_lookup.put(topic, id);
			topic_lookup.put(id, topic);
			
			if (Publisher){
				Publisher pub = node.createPublisher(topic, topic_type);
				publishers.put(id,pub);
			}
			else{
				Subscriber sub = node.createSubscriber(topic, topic_type, new MessageListenerRosSerial(this, id));
			}	
			return true;
		}
		catch (Exception e){
			e.printStackTrace();
			}
		return false;
	}
	
	
	 
	
	/*
	 * Blocking byte string read method.
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

				int chk=0;
				
				//Read topic id and add it to the checksum
				//little endian
				byte[] topic_id_b = new byte[2];
				read(topic_id_b);
				chk += topic_id_b[0]; chk+= topic_id_b[1];
				int topic_id = (int)(topic_id_b[1] <<  8) | (int)(topic_id_b[0]);

				//Read msg length (little endian) add it to checksum
				byte[] l_data_b = new byte[2];
				read(l_data_b);
				chk += l_data_b[0]; chk+= l_data_b[1];
				int l_data = (int)(l_data_b[1] <<  8) | (int)(l_data_b[0]);
				
				
				byte[] data = new byte[l_data]; //subtract one because the checksum isnt included...
				read(data);

				for(int i =0; i< l_data; i++) chk+= data[i];
				
				chk += istream.read(); //read in the last checksum byte
				
				if (chk%256 == 255){ //valid checksum
					switch(topic_id){
						case TOPIC_PUBLISHERS:
							{
							TopicInfo m =  new TopicInfo();
							m.deserialize(data);
							addTopic(m.topic_name, m.message_type, m.topic_id, true);
							break;
							}
						case TOPIC_SUBSCRIBERS:
							{
							TopicInfo m =  new TopicInfo();
							m.deserialize(data);
							addTopic(m.topic_name, m.message_type, m.topic_id, false);
							break;
							}
						case TOPIC_TIME:
							org.ros.message.Time t = node.getCurrentTime();
							org.ros.message.std_msgs.Time t_msg = new org.ros.message.std_msgs.Time();
							t_msg.data = t;
							send(TOPIC_TIME,t_msg);
							break;
						default:
      						Message msg = (Message) msg_classes.get(topic_id).newInstance();
							msg.deserialize(data);
							publishers.get(topic_id).publish(msg);
							break;
					}
				}
				else{
					System.out.println("Checksum failed!");
				}
			}
			catch(Exception e ){
				e.printStackTrace();
			}
		}
	}
	
	public void send(int id, org.ros.message.Message t){
		
		
		int l = t.serializationLength();
		
		byte[] packet_header = new byte[2+2+2]; //sync_flags + topic_id + data_len 
		packet_header[0] = (byte) 0xff;
		packet_header[1] = (byte) 0xff;
		packet_header[2] = (byte) id;
		packet_header[3] = (byte) (id >>8);
		packet_header[4] = (byte) l;
		packet_header[5] = (byte) (l >> 8);
		
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