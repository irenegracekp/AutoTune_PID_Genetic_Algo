dialect = mavlinkdialect("common.xml");
gcsNode = mavlinkio(dialect);
gcsPort = 14540;
connect(gcsNode,"UDP", 'LocalPort', gcsPort);

uavClient = mavlinkclient(gcsNode,1,1);
paramValueSub = mavlinksub(gcsNode,uavClient,'HEARTBEAT','NewMessageFcn', @(~,msg)disp(msg.Payload));


