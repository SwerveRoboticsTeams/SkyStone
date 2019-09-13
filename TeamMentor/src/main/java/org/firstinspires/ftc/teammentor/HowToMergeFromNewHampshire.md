Here are instructions for merging New Hampshire's updates into our repository.

You must follow these steps precisely or you could seriously mess up the club's repository!

To start, bring your local branches up to date.
1. Commit and push any local changes that you have on your machine
2. Pull the latest version of the Teams repository
3. Use the Fetch command to get the latest state for all of your remotes. Check all of the options in the Fetch dialog so you'll fetch tags etc.!
4. If you already have a local branch for master-swerve, delete it by right-clicking that branch and selecting Delete. You're not allowed to delete the current branch, so switch to another branch first by double-clicking that other branch.
5. If you already have a local branch for the New Hampshire master branch, delete it by right-clicking the branch and selecting Delete. 

IMPORTANT: If you have any merge errors or other problems at this point, you MUST sort them out before you proceed any further!


Next, bring the latest Swerve "master-swerve" branch onto your machine.
1. Expand REMOTES origin.
2. Double-click master-swerve to bring it onto your machine. 


Next, bring the latest New Hampshire changes onto your machine.

If you don't already have a remote to New Hampshire in your Source Tree window, right-click on REMOTES, select New Remote, and set:

    Remote Name: NewHampshire

    Remote URL: https://github.com/ftctechnh/ftc_app

1. The NewHampshire remote should now show in your left pane under REMOTES.
2. Expand the NewHampshire remote.
3. Double-click on the "master" branch to bring it onto your machine. I usually name it "NHMaster" so I'm not confused by other master branches.

Now, you're ready to merge!

If any of these steps result in a merge conflict, STOP IMMEDIATELY and get help from Steve Geffner or Bob Atkinson. 

1. Double-click on master-swerve so it's your current branch.
2. Right-click on NHMaster and select "Merge NHMaster into master-swerve"
3. Assuming all goes ok, commit and push. *You might need to get added to the permissions for master-swerve to do this step.*
4. Double-click on Teams so it's your current branch.
5. Right-click on master-swerve and select "Merge master-swerve into Teams"
6. Assuming all goes ok, commit BUT DO NOT push yet.
7. Go into Android Studio and Build..Make Project so that Android Studio will compile each of the team folders for this year. The purpose of this step is to see if the New Hampshire release has changed any APIs that have broken our code. If you get build errors, investigate them and fix them.
8. Commit any changes, and push.

You're done!





