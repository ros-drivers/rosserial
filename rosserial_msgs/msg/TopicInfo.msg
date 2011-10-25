# special topic_ids
uint16 ID_PUBLISHER=0
uint16 ID_SUBSCRIBER=1
uint16 ID_SERVICE_SERVER=2
uint16 ID_SERVICE_CLIENT=3
uint16 ID_PARAMETER_REQUEST=4
uint16 ID_LOG=5
uint16 ID_TIME=10

# The endpoint ID for this topic
uint16 topic_id

string topic_name
string message_type

# MD5 checksum for this message type
string md5_checksum

# size of the buffer message must fit in
int32 buffer_size
