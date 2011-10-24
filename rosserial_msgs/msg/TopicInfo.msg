#The topic_id defines the communication endpoint for this type of message
uint16 topic_id

uint16 ID_PUBLISHER=0
uint16 ID_SUBSCRIBER=1
uint16 ID_SERVICE_SERVER=2
uint16 ID_SERVICE_CLIENT=3
uint16 ID_PARAMETER_REQUEST=4
uint16 ID_LOG=5
uint16 ID_TIME=10

#any topic_id > 100 is a dynamically registered/advertised endpoint
#ie. it is the result of a subscribe, publish, or advertise

string topic_name

string message_type
