#include <iostream>
#include<iostream>
#include<fstream>
#include<string>

using namespace std;


int main() {
    string in_line;
	ifstream in("log.txt");
	while(getline(in,in_line)){
		cout<<in_line<<endl;
	}
	in.close();
}
