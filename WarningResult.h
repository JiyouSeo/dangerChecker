struct WarningResult
{
    bool isDanger;
    long fid;
    long sid;
    long firstObjectId;
    long secondObjectId;
    WarningResult(long _f,long _s, long _fo, long _so) : isDanger(false),fid(_f), sid(_s), firstObjectId(_fo),secondObjectId(_so) {};
    WarningResult() : isDanger(false), fid(-1),sid(-1),firstObjectId(-1),secondObjectId(-1) {};
};