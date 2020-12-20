#include <iostream>
#include<iostream>
#include<fstream>
#include<string.h>
#include<time.h>
#include "DangerChecker.cpp"
using namespace std;


int main() {
    string in_line;
	ifstream in("output.txt");
	char * delimeter = "\t";
	char str_buff[1000];
	string args[5];
	// 차례대로, 1,2,3번 cam 예측 시간 뒤에 3개는 1<->2,2<->3,1<->3 번 위험 예측 거리입니다. 
	DangerChecker dc(7,4,4,75,15,15); 
	clock_t start = clock();
	while(getline(in,in_line)) {

		int i = 0;
		strcpy(str_buff, in_line.c_str());
		char *ptr = strtok(str_buff, "\t\n\r ");
		while (ptr != NULL)
		{
			args[i] = ptr;
			i++;
			ptr = strtok(NULL, "\t\n\r ");
		}
		WarningResult result = dc.CheckDangerByID(stol(args[0]),stol(args[1]),stod(args[2]),stod(args[3]),stol(args[4]));
		if (result.isDanger) {
			cout << "frame = " << (float) stol(args[0]) << "\n";
		}
		dc.Flush();
	}
	clock_t end = clock();
	in.close();
	printf("Time: %lf\n", (double)(end - start) / CLOCKS_PER_SEC);
}