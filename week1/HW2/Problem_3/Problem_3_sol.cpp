#include <iostream>
#include <vector>
#include <climits>

using namespace std;

class Solution {
public:
    double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
        int m = nums1.size();
        int n = nums2.size();
        int mid = (m+n)/2;
        int median, last;
        int j = 0, k = 0;

        if(m+n == 0) return 0;
        if(n == 0) return (m % 2 == 1) ? nums1[mid] : (nums1[mid] + nums1[mid-1])/2.0;
        if(m == 0) return (n % 2 == 1) ? nums2[mid] : (nums2[mid] + nums2[mid-1])/2.0;

        for(int i = 0; i <= mid; i++)
        {
            last = median;
            int temp1 = (j < m) ? nums1[j] : INT_MAX;
            int temp2 = (k < n) ? nums2[k] : INT_MAX;

            if(temp1 > temp2)
                median = nums2[k++];
            else
                median = nums1[j++];
        }

        if((m+n)%2 == 1)
            return median;
        else
            return (median+last) / 2.0;
    }
};

int main() {
    vector<int> nums1, nums2;
    int num;
    char comma;

    cout << "Enter the first array: ";
    while (cin.peek() != '\n' && cin >> num) {
        nums1.push_back(num);
        if (cin.peek() == ' ')
            cin.ignore();
    }
    cin.ignore();

    cout << "Enter the second array: ";
    while (cin.peek() != '\n' && cin >> num) {
        nums2.push_back(num);
        if (cin.peek() == ' ')
            cin.ignore();
    }

    Solution solution;
    double median = solution.findMedianSortedArrays(nums1, nums2);
    cout << "The median is: " << median << endl;

    return 0;
}