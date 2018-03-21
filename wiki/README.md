# PR (Pull requests) Process

1. Once you have finished the changes to your branch in the local repo use the command: $git add [file_name] 
2. Repeat one for however many files you have modified for your changes
3. Commit all of your changes locally with the command: git commit -m "Some information about the commit."
4. If you continue to make changes you will have to readd modified files and re-commit them as well. 
5. Next please rebase/squash your changes before placing the pr.
6. Obtain your branches rebase has from the command: git merge-base [your branch] [desired branch]
7. Use the has in the following command: git rebase -i [hash]
8. You will then loaded into a file in terminal containing your commit history. Replace all pick's with squash except for the top pick.
9. Save the file and exit, another will open, save that one as well.
10. Run the command: git rebase [desired branch]
11. Now your commits have been rebased on top of master so they can easily be reverted if nessesary. Push your code to your remote branch.
12. Use the command: git push -u -f origin [your_branch] (For command 12 the -u is only needed for the first push to the remote branch.)
