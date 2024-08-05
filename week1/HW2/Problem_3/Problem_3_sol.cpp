#include <iostream>
#include <vector>
#include <climits>
#include <cmath>

using namespace std;

class Solution {
public:
    double findMedianSortedArrays(vector<int>& nums1, vector<int>& nums2) {
        if (nums1.size() > nums2.size())
            return findMedianSortedArrays(nums2, nums1);

        int m = nums1.size();
        int n = nums2.size();
        int totalLeft = (m+n+1)/2;
        int left = 0;
        int right = m;

        while(left < right)
        {
            int i = left + (right - left + 1)/2;
            int j = totalLeft - i;
            if(nums1[i-1] > nums2[j])
                right = i-1;
            else
                left = i;
        }

        int i = left;
        int j = totalLeft - i;
        int nums1LeftMax = (i == 0) ? INT_MIN : nums1[i-1];
        int nums1RightMin = (i == m) ? INT_MAX : nums1[i];
        int nums2LeftMax = (j == 0) ? INT_MIN : nums2[j-1];
        int nums2RightMin = (j == n) ? INT_MAX : nums2[j];

        if((m + n) % 2 == 1)
            return max(nums1LeftMax, nums2LeftMax);
        else
            return (double) (max(nums1LeftMax, nums2LeftMax) + min(nums1RightMin, nums2RightMin))/2.0;
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