# RobotPython2025
 Fresh start; created for the 2025 season; transition to python


 # FRC Robot Project

This repository contains the code for our FRC robot using Python, PyRobot, and WPILib.

## Setup

0. Create a python based profile in your VS Code: [optional profile file](RobotPy.code-profile)

1. Make sure you have Python 3.12.5
    ```sh
    python --version
    ------------------------ 
    OUTPUT: Python 3.12.5
1a. If not go to [Other Resources] and download it.

2. Clone the repository:
   ```sh
   git clone https://github.com/Team2556/RobotPython2025.git
   cd RobotPython2025
3. ??automatic for newcomer?? Activate the virtual environment:
    ```sh
    On Windows: 
    venv\Scripts\activate
    On macOS/Linux: 
    source venv/bin/activate
4. ??automatic for newcomer?? Install the dependencies:
    ```sh
    pip install . #the dot references your current directory
<!-- 

5. initialize robotpy
    ```sh

 3. Create folders named 'subsytems' and 'tests'
    ```sh
    mkdir subsytems
    mkdir tests
    py -3 -m robotpy init -->
<!--

7. move robot.py file created by initilization to 'src' folder
    move robot2.py src\ -->

4. Run the robot code:
    ```sh
    python src/robot.py

# Repository Structure
- We plan to use the following structure. (Or do we really?)
    <!-- ├── src/
    │   ├── ?maybe? __init__.py
    │   ├── robot.py
    ├── ?maybe? config/
    │   └── config.yaml
    -->
    ```sh
    FRC-Robot-Project/
    ├── robot.py
    ├── constants.py
    ├── subsystems/
    │   ├── __init__.py
    │   └── example_subsystem.py 
    ├── commands/
    │   ├── __init__.py
    │   └── example_command.py 
    ├── autonomous/
    │   ├── __init__.py
    │   └── example_command.py 
    ├── pathplanner/deploy/pathplanner/
    │   ├── autos
    │   └── paths
    ├── tests/
    │   ├── __init__.py
    │   └── test_robot.py
    ├── .gitignore
    ├── README.md
    ├── pyproject.toml
    └── venv/

# Other Resources
- Download python (3.12.5): https://www.python.org/downloads/windows/
- Python tutorial: https://docs.python.org/3.12/tutorial/index.html
- More instructions from WPI: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html
- deploy Python to RoboRIO: https://docs.wpilib.org/en/stable/docs/software/python/subcommands/deploy.html
- Command based programming: https://docs.wpilib.org/en/latest/docs/software/commandbased/what-is-command-based.html


# VS Code Extensions
TODO: Refer to our main github readme or sync here....
In the VS Code Extensions sidbar, these codes should help you find usefull extensions (well.. Git and Python are required)
- ms-python.python
- github.vscode-pull-request-github
- github.copilot
- ms-python.black-formatter
- njpwerner.autodocstring
- tamasfe.even-better-toml
- streetsidesoftware.code-spell-checker

## ORIGINAL Setup (FYI)

1. Clone the repository:
   ```sh
   git clone https://github.com/Team2556/RobotPython2025.git
   cd RobotPython2025
2. Create a vertual environment (from a terminal; in the repo directry on your pc)
    ```sh
    python -m venv venv
3. Activate the virtual environment:
    ```sh
    On Windows: 
    venv\Scripts\activate
    On macOS/Linux: 
    source venv/bin/activate
<!-- 4. Install the dependencies:
    ```sh
    pip install . #the dot references your current directory -->

5. initialize robotpy
    ```sh
    py -m robotpy init

<!-- 6. Create folders named 'src/subsytems' and 'tests'
    ```sh
    mkdir src/subsytems
    mkdir tests

7. move robot.py file created by initilization to 'src' folder
    move robot2.py src\ -->

6. Run the robotpy sync to get the RoboRIO python
    ```sh 
    py -m robotpy sync
8. Run the robot code:
    ```sh
    python src/robot.py

# Draft team plan

Below is a structured draft for the Radioactive Roaches Team Guide, designed for high school students and mentors. This guide outlines the team’s operational framework, progression ladder, new member program, and battle rhythm, incorporating your specific requirements. It is written to be clear, engaging, and actionable for both students and mentors.

Radioactive Roaches Team Guide
Welcome to the Radioactive Roaches, a high school robotics team dedicated to fostering creativity, technical skills, and leadership through hands-on robotics projects. This guide outlines how our team operates, how students grow through our Experience Ladder, and how we welcome and support new members. It serves as a roadmap for students, mentors, and parents to ensure a rewarding and organized experience.
Mission and Values
	•	Mission: Empower students to learn, innovate, and lead in robotics through collaboration and hands-on experience.
	•	Values: Growth, inclusivity, teamwork, and resilience.
	•	Team Charter: All members commit to respect, effort, and supporting each other’s growth.

Experience Ladder: Progression and Badges
Every student progresses through the Experience Ladder, a structured path that builds skills and responsibilities: Observer, Learner, Doer, Responsible. Progression is tracked for each student, and achievements are recognized with Dosimeter Badges, styled like dosimeters with increasing “dose counts” to reflect growth.
Levels and Behaviors
Each level has distinct behaviors, mentor engagement strategies, and expectations to encourage growth.
Level
Description
Typical Behaviors
Mentor Engagement
Observer
New to the team, watching and absorbing.
Asks basic questions, follows instructions, observes processes.
Ask: “Does that make sense?” Provide clear explanations, encourage curiosity.
Learner
Actively learning skills, starting to contribute.
Tries tasks with guidance, asks “why” and “how,” begins to suggest ideas.
Ask: “What do you think is next?” Guide through tasks, encourage problem-solving.
Doer
Independently performs tasks, teaches others.
Executes tasks confidently, helps peers, takes initiative on projects.
Ask: “Who are you going to teach that to?” Encourage leadership and peer mentoring.
Responsible
Leads projects, drives team success.
Plans tasks, delegates, ensures project quality, mentors lower levels.
Ask: “What does this project need to really succeed?” Support strategic thinking.
Tracking and Badges
	•	Tracking: Mentors maintain a shared spreadsheet or database to log each student’s current level, skills mastered, and contributions. Regular check-ins (monthly or per project milestone) assess progress.
	•	Dosimeter Badges:
	◦	Observer: Single bar (low dose).
	◦	Learner: Two bars.
	◦	Doer: Three bars.
	◦	Responsible: Four bars (max dose).
	•	Badges are awarded at team ceremonies (e.g., Induction Ceremony, end-of-season celebration) to celebrate growth.
	•	Students may be at different levels across sub-teams (e.g., Learner in Mechanical, Observer in Programming).
Mentor Guidance
	•	Assess Progress: Use behaviors as a rubric. Discuss with students to set goals for the next level.
	•	Encourage Growth: Tailor tasks to stretch students just beyond their current level (e.g., pair Observers with Doers for shadowing).
	•	Celebrate Milestones: Recognize badge awards publicly to motivate the team.

New Member Program
The New Member Program welcomes students and parents, sets expectations, and integrates new members into the team. It aligns with the school year and emphasizes inclusivity, focusing on fit with the team charter rather than aptitude.
Timeline and Key Events
	1	Tryouts/Interviews (Early September):
	◦	Purpose: Introduce the team, assess interest, and determine primary/secondary sub-team placements (e.g., Mechanical, Programming, Strategy, Outreach).
	◦	Process:
	▪	Brief interviews with students and parents to discuss goals and commitment.
	▪	Focus on alignment with team charter (e.g., willingness to learn, teamwork).
	▪	Not aptitude-based; all interested students are welcome.
	▪	Collect preferences for sub-teams to guide initial placements.
	◦	Outcome: Draft sub-team assignments, shared with families.
	2	Introduction Faire and Parent Meeting (Mid-September):
	◦	Purpose: Set expectations, finalize sub-team placements, and build community.
	◦	Activities:
	▪	Showcase sub-teams through demos and student presentations.
	▪	Discuss team rules, schedule, and commitment (e.g., build season intensity).
	▪	Parents and students sign the Team Rules and Contract (covering behavior, safety, and participation).
	▪	Allow students to refine sub-team preferences based on Faire exposure.
	◦	Outcome: Finalized sub-team placements, signed contracts, and engaged families.
	3	Orientation Week (Late September):
	◦	Purpose: Deepen understanding of sub-teams and team culture.
	◦	Activities:
	▪	Detailed sub-team demos and hands-on mini-projects (e.g., assemble a simple mechanism, write basic code).
	▪	Team-building activities to foster bonds.
	▪	Assign each new member a Buddy/Journeyman (a Doer or Responsible student) for ongoing support.
	◦	Outcome: New members feel connected and ready to engage as Observers.
	4	Monthly Crash Courses (October–December):
	◦	Purpose: Build skills and confidence through student-led learning.
	◦	Format: 1-hour sessions on topics like CAD, soldering, scouting, or outreach, developed and taught by Doers/Responsible students.
	◦	Outcome: Observers gain exposure to diverse skills, preparing them for Learner status.
	5	Induction Ceremony (December, Pre-Build Season):
	◦	Purpose: Celebrate new members’ integration and progression.
	◦	Activities:
	▪	Assess Observers for Learner status based on engagement and crash course participation.
	▪	Award Dosimeter Badges (most new members receive Learner badge).
	▪	Share team vision for the upcoming build season.
	◦	Outcome: New members officially join as Learners, ready for build season.

Battle Rhythm
The Battle Rhythm outlines recurring team activities, ensuring structure and progress throughout the year, with heightened intensity during build season (January–March).
Year-Round Activities
	•	Weekly All-Team Meeting (1–2 hours):
	◦	Review progress, share sub-team updates, and align on goals.
	◦	Include design reviews or report-outs to keep everyone informed.
	◦	Open Q&A for Observers to build understanding.
	•	Monthly Crash Courses: Continue as described in the New Member Program.
	•	Sub-Team Meetings: Scheduled by sub-team leads (Responsible students or mentors) for focused work.
Build Season (January–March)
	•	Kickoff (Early January):
	◦	Game rules released. All members participate in brainstorming and design review to ideate solutions.
	◦	Form cross-sub-team groups to analyze rules and propose strategies.
	•	Prototyping (Weeks 1–2):
	◦	Entire team contributes to prototyping (e.g., testing mechanisms, coding basic functions).
	◦	Observers shadow, Learners assist, Doers/Responsible lead.
	•	Intensified Schedule:
	◦	First 2 Weeks: All-team meetings/design reviews every other day to iterate designs rapidly.
	◦	Weeks 3–6: Revert to weekly all-team meetings, with daily sub-team work.
	•	Project Assignments:
	◦	Observers: Support prototyping, observe competition robot work.
	◦	Learners: Work on alternative projects (e.g., practice robots, outreach displays) and document protocols.
	◦	Doers/Responsible: Build and refine the competition robot.
	•	Design Reviews: Weekly (or more frequent early on) to critique prototypes, finalize designs, and assign tasks.
Competition Season (March–April)
	•	Observers/Learners:
	◦	Assist with scouting (e.g., observing opponent robots, collecting data).
	◦	Lead cheering and team spirit activities.
	•	Doers/Responsible:
	◦	Operate and maintain the competition robot.
	◦	Mentor Observers/Learners in scouting roles.
	•	All Members: Participate in pit setup, strategy discussions, and representing the team to judges and visitors.

Team Structure and Sub-Teams
	•	Sub-Teams: Mechanical, Programming, Strategy/Scouting, Outreach (adjust based on team needs).
	•	Leadership:
	◦	Student Leads (Responsible level): Oversee sub-teams, report at all-team meetings.
	◦	Mentors: Guide sub-teams, assess progression, and ensure safety.
	•	Buddy System: Pairs new members with experienced students for mentorship and integration.

Team Rules and Contract
	•	Commitment: Attend meetings, communicate absences, and contribute to team goals.
	•	Respect: Treat all members, mentors, and competitors with kindness and professionalism.
	•	Safety: Follow shop and competition safety protocols (e.g., wear safety glasses, handle tools properly).
	•	Growth Mindset: Embrace challenges, ask questions, and support peers’ learning.
	•	Signed By: Students and parents at the Introduction Faire.

Tracking and Evaluation
	•	Progression Tracking: Mentors update the Experience Ladder database monthly, noting skills, contributions, and level changes.
	•	Feedback: Bi-monthly mentor-student check-ins to discuss goals and growth.
	•	Badge Awards: Presented at Induction Ceremony, end-of-season celebration, or as earned during build season.
	•	Sub-Team Placements: Reviewed mid-season to allow shifts based on interest or skill development.

Key Dates (Example for 2025–2026 School Year)
	•	September 1–10: Tryouts/Interviews.
	•	September 15: Introduction Faire and Parent Meeting.
	•	September 20–24: Orientation Week.
	•	October–December: Monthly Crash Courses.
	•	December 15: Induction Ceremony.
	•	January 4: Build Season Kickoff.
	•	January–February: Intensified build schedule.
	•	March–April: Competition season.

For Mentors: Supporting Student Growth
	•	Engage by Level: Use the mentor engagement questions to tailor interactions.
	•	Model Behaviors: Demonstrate curiosity, collaboration, and leadership.
	•	Provide Opportunities: Assign tasks that challenge students to stretch (e.g., Observers watch a weld, Learners try one with supervision).
	•	Celebrate Effort: Recognize both successes and persistence to build confidence.
	•	Communicate with Parents: Share progress updates at key milestones (e.g., Induction Ceremony).

For Students: Your Journey
	•	Own Your Growth: Ask questions, seek feedback, and take on new challenges.
	•	Support Your Teammates: Share knowledge and cheer each other on.
	•	Earn Your Dosimeter: Each badge reflects your hard work and leadership.
	•	Have Fun: Robotics is about creativity and community—enjoy the ride!

This guide is a living document and will evolve with team feedback. Let’s make this season radioactively awesome!

Notes for Implementation
	•	Document Format: Create this as a shared digital document (e.g., Google Doc, Notion) for easy access. Include visuals of Dosimeter Badges and a calendar of key dates.
	•	Badge Design: Collaborate with the Outreach sub-team to design Dosimeter Badges (e.g., 3D-printed or embroidered patches).
	•	Database Setup: Use a simple tool like Google Sheets or Airtable to track student progression, with columns for name, sub-team, level, skills, and notes.
	•	Parent Communication: Share a condensed version of the guide with parents at the Introduction Faire, highlighting expectations and key dates.
	•	Student Input: Involve Doers/Responsible students in refining crash course topics and ceremony planning to build ownership.
Would you like me to generate a visual for the Dosimeter Badges, create a sample tracking spreadsheet, or refine any section further (e.g., add specific crash course topics or sub-team descriptions)?
