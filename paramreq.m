dialect = mavlinkdialect("common.xml");
gcsNode = mavlinkio(dialect);
gcsPort = 14540;
connect(gcsNode,"UDP", 'LocalPort', gcsPort);

uavClient = mavlinkclient(gcsNode,1,1);
paramValueSub = mavlinksub(gcsNode,uavClient,'PARAM_VALUE','BufferSize',10,'NewMessageFcn', @(~,msg)disp(msg.Payload));

Atti_1 = mavlinksub(gcsNode,uavClient,'ATTITUDE','NewMessageFcn', @(~,msg)disp(msg.Payload));
Atti_2 = mavlinksub(gcsNode,uavClient,'ATTITUDE_TARGET','NewMessageFcn', @(~,msg)disp(msg.Payload));
 

