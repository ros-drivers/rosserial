/*
 * publisher.h
 *
 *  Created on: Aug 5, 2011
 *      Author: astambler
 */

#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include "node_output.h"

namespace ros{
  /* Generic Publisher */

  class Publisher
  {
    public:
      Publisher( const char * topic_name, Msg * msg ): topic_(topic_name), msg_(msg){};
      int publish( Msg * msg ){
    	  return no_->publish(id_, msg_);
      };

      const char * topic_;

      Msg *msg_;
      int id_;
      NodeOutput_* no_;

  };

}


#endif /* PUBLISHER_H_ */
