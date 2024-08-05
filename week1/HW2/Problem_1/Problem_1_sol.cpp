#include <iostream>
using namespace std;
int BinarySearch(int *, int, int, int);

int main()
{
    cout << "number of elements in the array: ";
    int n;
    cin >> n;

    int* nums = new int[n];
    cout << "enter the array: ";
    for (int i = 0; i < n; ++i) cin >> nums[i];

    cout << "target value: ";
    int target;
    cin >> target;
    
    int length = n;
    int start = BinarySearch(nums, 0, length - 1, target);
    int end = start;

    if(start == 777777777)
    {
        cout << "\nOutput: [-1, -1]" << endl;
        return 0;
    }
    while(1)
    {
        int temp = BinarySearch(nums, 0, start - 1, target);
        if(temp == 777777777) break;
        start = temp;
    }
    while(1)
    {
        int temp = BinarySearch(nums, end + 1, length - 1, target);
        if(temp == 777777777) break;
        end = temp;
    }
    cout <<"\nOutput: [" << start << ", " << end << "]" << endl;
    return 0;
}

int BinarySearch(int *nums, int left, int right, int target)
{
    if(left <= right)
    {
        int mid = left + (right - left) / 2;

        if(nums[mid] == target) return mid;
        if(target > nums[mid]) return BinarySearch(nums, mid + 1, right, target);
        return BinarySearch(nums, left, mid - 1, target);
    }
    return 777777777;   // all numbers in the array should be less than 10^9
}