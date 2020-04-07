
def statusStringToDictionary(status):
    print(status.strip())
    splitStatus=status.strip().split(';')
    statusDictionary={}
    for i in splitStatus:
        if(i!=''):
            splitValue=i.split(':')
            print(splitValue)
            statusDictionary[splitValue[0]]=float(splitValue[1])
    return statusDictionary


print(statusStringToDictionary("pitch:5;roll:5;yaw:3;vgx:4;vgy:5;vgz:1;templ:30;temph:40;tof:12;h:3;bat:60;baro:55.2;time:20;agx:2.2;agy:3.3;agz:4.4;\r\n"))
