/*
 * service_server.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */


#ifndef SERVICE_SERVER_H_
#define SERVICE_SERVER_H_


#include "node_output.h"

namespace ros{
template<typename SrvRequest , typename SrvResponse>
  class ServiceServer : MsgReceiver{
  public:
  	typedef void(*CallbackT)(const SrvRequest&,  SrvResponse&);

  private:
  	CallbackT cb_;

  public:
  	ServiceServer(const char* topic_name, CallbackT cb){
  		this->topic_ = topic_name;
  		this->cb_ = cb;
  	}

  	ServiceServer(ServiceServer& srv){
  		this->topic_ = srv.topic_;
  		this->cb_ = srv.cb_;
  	}
  	virtual void receive(unsigned char * data){
  		req.deserialize(data);
  		this->cb_(req, resp);
  		no_->publish(id_, &resp);
  	}

	virtual int _getType(){
		return 3;
	}
	virtual const char * getMsgType(){
		return req.getType();
	}

  	SrvRequest req;
  	SrvResponse resp;
  	NodeOutput_ * no_;

  };
}

#endif /* SERVICE_SERVER_H_ */
