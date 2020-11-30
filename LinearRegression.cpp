// C++ program to find the 
// regression line 
#include<bits/stdc++.h> 
#include<deque>
using namespace std; 
//https://www.geeksforgeeks.org/least-square-regression-line/
// Function to calculate b 
double calculateB(deque<double> x, deque<double> y, int n) 
{ 
    // sum of array x 
    double sx = accumulate(x.begin(),x.end(), 0.0); 
  
    // sum of array y 
    double sy = accumulate(y.begin(),y.end(), 0.0); 
  
    // for sum of product of x and y 
    double sxsy = 0; 
  
    // sum of square of x 
    double sx2 = 0; 
    for(int i = 0; i < n; i++)  
    { 
        sxsy += x[i] * y[i]; 
         sx2 += x[i] * x[i]; 
    } 
    double b = (double)(n * sxsy - sx * sy) / 
                       (n * sx2 - sx * sx); 
  
    return b; 
} 
  
// Function to find the 
// least regression line 
// if whatCoordinate == 0, then coordinate X, whatCoordinate == 1, then coordinate Y
pair<double,double> leastRegLine(deque<double> X, deque<double> Y, int n,int whatCoordinate) 
{ 
  
    // Finding b 
    double b = calculateB(X, Y, n); 
  
    double meanX = accumulate(X.begin(),X.end(), 0.0) / n; 
    double meanY = accumulate(Y.begin(),Y.end(), 0.0) / n; 
  
    // Calculating a 
    double a = meanY - b * meanX; 
  
    // Printing regression line 
    // cout << ("Regression line:") << endl; 
    // cout << ("Y = "); 
    // printf("%.3f + ", a); 
    // printf("%.3f *X\n", b); 
    return pair<double,double>(a,b);
} 


  
// This code is contributed by PrinciRaj1992  
