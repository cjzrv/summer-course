# Problem 3

> 這是使用命令行作為 I/O 界面編寫的版本。若需查看 LeetCode 格式（作為函數被調用）的程式碼，可以在 /week1/HW2/leetcode_version 目錄下找到。

### Description
Given two sorted arrays nums1 and nums2 of size m and n respectively, return the median of the two sorted arrays.
The overall run time complexity should be O(log (m+n)).

### Explanation
看到複雜度要求要 O(log(n))，我第一個想法就是用 binary search 的方式，想辦法在不合併兩個陣列的情況下找出中位數，然而寫了五個小時還是想不出具體的解法，這題我放棄。

這題我提交的程式碼是照搬下面這個連結裡的影片內容。  
https://leetcode.cn/problems/median-of-two-sorted-arrays/solutions/258842/xun-zhao-liang-ge-you-xu-shu-zu-de-zhong-wei-s-114/

這位作者定義了一個虛擬的分隔線，使兩個陣列各被切割為左右兩部份，將被切割出來的兩陣列左半部份視為一個集合，右半部份視為另一個集合，兩個集合的元素數量相等（奇數時定義為左集合多 1），且右半部份的所有元素都大於左半部份。

而找出這個分隔線的方法事先將分隔線設在中間，再透過分隔線左邊的最大數值必須小於右邊最小值這個條件不斷對分隔線進行調整（每次調整都是用 binary search的方式），直到找出符合條件的正確分隔線。

找到這條分隔線後，若所有元素的總數為奇數，則中位數是左集合最大的數，若元素總數為偶數，則中位數是左集合最大數及右集合最小數的平均值。

### Time Complexity

只需在較短的陣列中進行 binary search，故時間複雜度為 O(min(m, n))。

### How to run ~~my code~~ this code

編譯並執行後，在命令行界面輸入要處理的兩個陣列，便會輸出結果在命令行界面上。

<img src="./Screenshot_problem_3.png" alt="result" width="800px"/>