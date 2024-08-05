# Problem 1

> 這是我使用命令行作為 I/O 界面編寫的版本。若需查看 LeetCode 格式（作為函數被調用）的程式碼，可以在 /week1/HW2/leetcode_version 目錄下找到。

### Description
Given an array of integers nums sorted in non-decreasing order, find the starting and ending position of a given target value.If target is not found in the array, return [-1, -1]. You must write an algorithm with O(log n) runtime complexity.

### Explanation
我使用 binary search 先找到 target number 所在的任意一個 index，再對該位置的前半部份（0 到 index-1）用 binary search 尋找有無其他 target number，不斷重複此步驟直到找不到 target number 為止，如此便能找出 target number 起始的 index。

而後，改對一開始找到的 index 的後半部份（index+1 到 length-1）用相同於上面的方式不斷往後以 binary search 尋找有無其它 target number，如此便能找出 target number 結尾的 index。

### Time Complexity
每次進行 binary search 都能砍掉一半的搜尋範圍，故時間複雜度為 O(log n)。

### How to run my code
編譯並執行後，在命令行界面根據提示輸入陣列大小、陣列內容和目標數字，目標數字在該陣列的起始及結束位置便會輸出在命令行界面上。

<img src="./Screenshot_problem_1.png" alt="result" width="800px"/>