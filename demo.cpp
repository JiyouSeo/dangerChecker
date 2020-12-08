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
	DangerChecker dc;
	clock_t start = clock();
	while(getline(in,in_line)) {

		int i = 0;
		strcpy(str_buff, in_line.c_str());
		char *ptr = strtok(str_buff, "\t\n\r ");    //구분자는 "\t\n "입니다.
		while (ptr != NULL)
		{

			args[i] = ptr;
			i++;
			ptr = strtok(NULL, "\t\n\r ");
		}
		bool isDanger = dc.CheckDangerByID(stol(args[0]),stol(args[1]),stod(args[2]),stod(args[3]),stol(args[4]));
		if (isDanger) {
			cout << "time = " << (float) stol(args[0]) / 30 << "\n";
		}
		dc.Flush();
	}
	clock_t end = clock();
	in.close();
	printf("Time: %lf\n", (double)(end - start) / CLOCKS_PER_SEC);
}