struct WarningResult
{
    /* 
    isDanger : false 면 경고가 아니고 true 면 경고인 프레임임을 보입니다.
    fid,sid : first object id 와 second object id 를 의미합니다. 만약 경고상황이 아니면 둘다 -1입니다.
    firstObjectId, secondObjectId : 각 객체의 object id 입니다. 사람인지 차인지 기타인지를 구별가능합니다.
    */
    bool isDanger;
    long fid;
    long sid;
    long firstObjectId;
    long secondObjectId;
    WarningResult(long _f,long _s, long _fo, long _so) : isDanger(false),fid(_f), sid(_s), firstObjectId(_fo),secondObjectId(_so) {};
    WarningResult() : isDanger(false), fid(-1),sid(-1),firstObjectId(-1),secondObjectId(-1) {};
};