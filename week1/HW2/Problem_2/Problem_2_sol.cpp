#include <string>
#include <iostream>
using namespace std;

string s;
int range[2] = {0, 0};
int biggest = 1;

void Even(int, int);
void Odd(int, int);

int main() {
    cout << "Enter the string: ";
    cin >> s;
    for (int i = 0; i < s.length()-1; i++)
    {
        Even(i, 0);
        Odd(i, 0);
    }
    cout << "Output: ";
    for(int k = range[0]; k <= range[1]; k++) cout << s[k];
    cout << endl;
    return 0;
}

void Even(int i, int size)
{
    if((i-size) < 0 || (i+size+1) > s.length()) return;

    if(s[i-size] == s[i+size+1])
    {
        if(size*2+2 > biggest){
            biggest = size*2+2;
            range[0] = i-size;
            range[1] = i+size+1;
        }
        Even(i, size+1);
    }
}

void Odd(int i, int size)
{
    if((i-size-1) < 0 || (i+size+1) > s.length()) return;
    
    if(s[i-size-1] == s[i+size+1])
    {
        if(size*2+3 > biggest){
            biggest = size*2+3;
            range[0] = i-size-1;
            range[1] = i+size+1;
        }
        Odd(i, size+1);
    }
}