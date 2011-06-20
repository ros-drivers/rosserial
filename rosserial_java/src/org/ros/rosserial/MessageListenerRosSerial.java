package org.ros.rosserial;

import org.ros.MessageListener;
import org.ros.message.Message;
import org.ros.rosserial;

public class MessageListenerRosSerial<MessageType> implements MessageListener<MessageType>{
	
	RosSerial rs;
	int id;
	int tt;
	
	public MessageListenerRosSerial(RosJSerial port, int topic_id, int topic_type){
		rs= port;
		id= topic_id;
		tt = topic_type; 
	}
	
	public void onNewMessage( MessageType t) {
		rs.send(id, tt, (Message) t);
      }
}
