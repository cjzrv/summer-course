#include <vector>
using namespace std;

class Solution {
public:
    vector<int> searchRange(vector<int>& nums, int target) {
        
        int length = nums.size();
        int start = BinarySearch(nums, 0, length - 1, target);
        int end = start;

        if (start == 777777777) {
            return {-1, -1 };
        }
        while (1) {
            int temp = BinarySearch(nums, 0, start - 1, target);
            if (temp == 777777777)
                break;
            start = temp;
        }
        while (1) {
            int temp = BinarySearch(nums, end + 1, length - 1, target);
            if (temp == 777777777)
                break;
            end = temp;
        }
        return { start, end };
    }
    int BinarySearch(vector<int>& nums, int left, int right, int target) {
        if (left <= right) {
            int mid = left + (right - left) / 2;

            if (nums[mid] == target)
                return mid;
            if (target > nums[mid])
                return BinarySearch(nums, mid + 1, right, target);
            return BinarySearch(nums, left, mid - 1, target);
        }
        return 777777777; // all numbers in the array should be less than 10^9
    }
};