mercury planner manual installation
===================================

This planner was used in ipc 2014, and scored high in the transport domain, hence we use it
for robocup at work, which has a similar domain.

The following instructions were tested under ubuntu 14.04 , 64 bit PC and are ros independant

1. Download the planner

        wget https://helios.hud.ac.uk/scommv/IPC-14/repo_planners/Mercury-fixed.zip

2. Install dependencies

        sudo apt-get install bison flex gawk g++-multilib pypy
        
3. Unzip only sequential satisfactory (seq-sat-mercury)

        unzip Mercury-fixed.zip -x seq-agl-mercury.tar.gz && tar -xf seq-sat-mercury.tar.gz

4. Compile the program:

        cd seq-sat-mercury
        ./build

wait unitl is done... (about 3 min on a i7 PC)

5. test the planner:

provide with domain.pddl and problem.pddl (problem pddl definition) at the root of the directory.

        ./plan domain.pddl problem.pddl plan

if you dont have a domain.pddl and problem.pddl there is one simple example at the end of this wiki.
        
6. Done! Now you should be able to see many plans happening, with different heuristics,

you might have to stop the last heuristics because they take a long time or even hang apparently.

for the example at the end of this wiki the output looks like this:

        .
        .
        .
        clean ghost loca (1)
        move ghost loca locb (1)
        clean ghost locb (1)
        move ghost locb loca (1)
        Plan length: 4 step(s).
        .
        .
        .
        etc...


domain.pddl example
===================

(define (domain cleaning_robot)

  (:requirements
    :typing
  )

  (:types
    location
    robot
  )

  (:predicates
    (at ?r - robot ?l - location)       ; robot r? is at location l?
    (clean ?l - location)                       ; location ?l is clean
  ) 

  (:action move
    :parameters (?r - robot ?source ?destination - location)
    :precondition (at ?r ?source)
    :effect (and    ( not (at ?r ?source))
                                (at ?r ?destination)
                )
  )

  (:action clean
    :parameters (?r - robot ?l - location)
    :precondition (and (at ?r ?l) (not(clean ?l)))
    :effect (clean ?l)
  )
)

problem.pddl example
====================

(define (problem p01)
        
  (:domain cleaning_robot)

  (:objects 
        ghost - robot
        locA locB - location
  )

  (:init 
        (at ghost locA)
        (not(clean locA))
        (not(clean locB))
        
  ) 

  (:goal
        (  and  (at ghost locA)
                (clean locB)
                (clean locA)
        )
  )
)
