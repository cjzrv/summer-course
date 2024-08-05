#include <string>
#include <iostream>

using namespace std;

class Solution
{
public:
    int range[2] = {0, 0};
    int biggest = 1;
    string temp;
    string longestPalindrome(string s)
    {
        temp = s;
        int i = 0;
        for (i = 0; i < s.length() - 1; i++)
        {
            Even(i, 0);
            Odd(i, 0);
        }
        return s.substr(range[0], range[1]-range[0]+1);
    }

        void Even(int i, int size)
        {
            if ((i - size) < 0 || (i + size + 1) > temp.length())
                return;

            if (temp[i - size] == temp[i + size + 1])
            {
                if (size*2 + 2 > biggest)
                {
                    biggest = size * 2 + 2;
                    range[0] = i - size;
                    range[1] = i + size + 1;
                }
                Even(i, size + 1);
            }
        }

        void Odd(int i, int size)
        {
            if ((i - size - 1) < 0 || (i + size + 1) > temp.length())
                return;

            if (temp[i - size - 1] == temp[i + size + 1])
            {
                if (size*2 + 3 > biggest)
                {
                    biggest = size*2 + 3;
                    range[0] = i - size - 1;
                    range[1] = i + size + 1;
                }
                Odd(i, size + 1);
            }
        }
};