#include <map>
#include <string>
#include <vector>
#include "ObjectLog.h"

using namespace std;
/*frame ID X Y objectID */
class DangerChecker
{
private:
    /* data */
    map<int,vector<ObjectLog>> centerPointEachID;
    map<int,CenterPoint> futurePointEachID;
    map<int,vector<double>> parametersEachID;
public:
    DangerChecker(/* args */);
    /*def point_check(self, id, center, fcnt, frame, color)*/
    int CheckDangerByID(long frame,long id,long double X,double Y,long objectID); 
    /*def save_point(self, id, center, fcnt, num=config["NUMBER_OF_CENTER_POINT"])*/
    void SavePointEachID(long frame,long id,long double X,double Y,long objectID);
    /*def least_square(self, id)*/
    vector <int> GetParamsByLSM(long id);
    ~DangerChecker();
};

DangerChecker::DangerChecker(/* args */)
{
}

DangerChecker::~DangerChecker()
{
}

int DangerChecker::CheckDangerByID(long frame,long id,long double X,double Y,long objectID) {
    return 0;
}
void DangerChecker::SavePointEachID(long frame,long id,long double X,double Y,long objectID) {
    if (centerPointEachID.find(id) == centerPointEachID.end()) {
        // 존재하지 않음
        ObjectLog ol(frame,X,Y,objectID);
        centerPointEachID[id].push_back(ol);
        // parameter 와 future point 의 init 필요한 상태
        

    }
    else {
        // 존재
    }
    
}
vector <int> DangerChecker::GetParamsByLSM(long id) {

}