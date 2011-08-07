/*
 * NodeOutput.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */

#ifndef NODEOUTPUT_H_
#define NODEOUTPUT_H_

/*
 * This class is responsible for controlling the node ouput.
 * It it is the object that is passed to Publishers and services
 */

#include "msg.h"

namespace ros{

	class NodeOutput_{
	public:
		virtual int publish(int id, Msg* msg)=0;
	};


	template<class Hardware, int OUTSIZE =512>
	class NodeOutput : public NodeOutput_{

	private:
		Hardware* hardware_;
		bool configured_;
	    unsigned char message_out[OUTSIZE];

	public:
	    NodeOutput(Hardware* h){
	    	hardware_ = h;
	    	configured_ = false;
	    }
	    
	    NodeOutput(){};
	   void setHardware(Hardware* h){
			hardware_  = h;
			configured_=false;
	   }
	   
		void setConfigured(bool b){
			configured_ =b;
		}
		bool configured(){return configured_;};

		virtual int  publish(int id, Msg * msg){
				if(!configured_) return 0;

				/* serialize message */
				int l = msg->serialize(message_out+6);

				/* setup the header */
				message_out[0] = 0xff;
				message_out[1] = 0xff;
				message_out[2] = (unsigned char) id&255;
				message_out[3] = (unsigned char) id>>8;
				message_out[4] = (unsigned char) l&255;
				message_out[5] = ((unsigned char) l>>8);

				/* calculate checksum */
				int chk = 0;
				for(int i =2; i<l+6; i++)
				  chk += message_out[i];
				message_out[6+l] = 255 - (chk%256);

				hardware_->write(message_out, 6+l+1);
				return 1;
			  }
	};
}
#endif /* NODEOUTPUT_H_ */
