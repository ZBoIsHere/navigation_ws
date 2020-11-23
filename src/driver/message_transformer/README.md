## MESSAGE_TRANSFORMER

message_tranformer is a python node to do two-way convertion between udp message and rostopic which is driven by configuration file.


## RUN
`rosrun message_transformer message_transformer.py`

## rule.json的配置规则

message_tranformer可进行UDP和ros topic的双向转换.
### struct format
rule.json中的数据类型采用python struct format,format类型表如下：
|  格式符   | C语言类型  | Python类型 | Standard size |
|  ----  | ----  | --- | --- |
| x  | 	pad byte(填充字节) | no value | |
| c |	char |	string of length 1 |	1 |
|b|	signed char|	integer|	1|
|B|	unsigned char|	integer|	1
|?|	_Bool|	bool|	1
|h|	short|	integer|	2
|H|	unsigned short|	integer|	2
|i|	int|	integer|	4
|I(大写的i)|	unsigned int|	integer|	4
|l(小写的L)|	long|	integer|	4
|L|	unsigned long|	long|	4
|q|	long long|	long|	8
|Q|	unsigned long long|	long|	8
|f|	float|	float|	4
|d|	double|	float|	8
|s|	char[]|	string|	 
|p|	char[]|	string|	 
|P|	void *|	long|
### topic to udp
topic到udp的范例如下:
```json
{
    "convert":{
        "src":"topic",
        "des":"udp"
    },
    "topic_name":"arm_center_mass",
    "msg_pkg": "std_msgs.msg",
    "msg_type": "Float32MultiArray",
    "mapping":{
        ".data[0]": "d",
        ".data[1]": "d",
        ".data[2]": "d",
        ".data[3]": "d"
    },
    "command_code": 100
}
```
其中:  
`topic_name`为订阅的ros-topic名.  
`msg_pkg`为topic对应的msg所在的python包.  
`msg_type`为topic对应的python 数据结构类型.  
`command_code`对应udp packet中的指令码.  
`mapping`描述了topic字段到udp packet间的映射关系,转发程序会定义一个msg对象，在上面的范例中，会将msg.data[0]映射为一个double型的变量并存在一段内存中，待发送的udp内存按照`mapping`的定义顺序将数据叠加起来，如上例中发送的内存前8字节为msg.data[0],接下来为msg.data[1],再接下来为msg.data[2]等.该内存将复制到udp CommandHead后的parameters中，位于CommandHead开始后的第12个字节后.
### udp to topic 
udp到topic的范例如下:
```json
{
    "convert":{
        "src":"udp",
        "des":"topic"
    },
    "local_port": 43897,
    "topic_name":"joint_angle",
    "msg_pkg": "std_msgs.msg",
    "msg_type": "Float64MultiArray",
    "type_mapping":{
        ".data[0]": "d",
        ".data[1]": "d",
        ".data[2]": "d",
        ".data[3]": "d",
        ".data[4]": "d",
        ".data[5]": "d",
        ".data[6]": "d",
        ".data[7]": "d",
        ".data[8]": "d",
        ".data[9]": "d",
        ".data[10]": "d",
        ".data[11]": "d"
    },
    "offset_mapping":{
        ".data[0]": 0,
    ".data[1]": 8,	
        ".data[2]": 16,
    ".data[3]": 24,    
        ".data[4]": 32,
    ".data[5]": 40,
        ".data[6]": 48,
    ".data[7]": 56,
        ".data[8]": 64,
    ".data[9]": 72,
        ".data[10]": 80,
    ".data[11]": 88
    },
    "pre_cmd": "print(\"pre_cmd\")",
    "pos_cmd": "msg.data=[0.0]*12",
    "command_code": 2306
}
```
其中：  
`topic_name`为订阅的ros-topic名.  
`msg_pkg`为topic对应的msg所在的python包.  
`msg_type`为topic对应的python 数据结构类型.  
`command_code`对应udp packet中的指令码.  
对于从udp接受到的报文，根据`offset_mapping`来确定从内存偏移多少开始的数据为消息的某一成员,`type_mapping`则用来将从该内存偏移起始的数据映射为某一类型的数据赋值给msg成员.在上面的范例中，会将从udp包接收的内存，从第0字节开始解析出一个double赋值给msg.data[0]，由于已解析出一个double,会将从第8个字节开始的内存解析为data[2],以此类推.待解析udp内存的起点为udp CommandHead后的parameters中，位于CommandHead开始后的第12个字节后.  
`local_port`为本地接收数据的udp端口.  
`pre_cmd`和`pos_cmd`为两条python指令，分别在实例化msg之前和之后执行,如可`pos_cmd`中对消息的时间戳进行赋值等.  