#! /bin/bash

# mercury planner bash script
# this script calls a three step pipeline, which at the end generates a plan using IPC 2014 mercury planner:
# the steps are : translate, preprocess and search
# translate inputs domain.pddl problem.pddl and generates output.sas file
# preprocess inputs output.sas and generates output
# search inputs 'output' (file) and generates mercury.plan
# for script usage please call this script with --help or --h as argument

log_mercury(){
printf '[Mercury_planner] : '
for each in $@
do
    printf $each
    printf ' '
done
printf '\n'
}


# --------------CHECK ARGS--------
function show_help_and_exit(){
    printf 'Mercury planner script usage :\n\n'
    echo '1 argument = path to domain'
    echo '2 argument = path to pddl problem definition'
    echo '3 argument = path to mercury_planner binaries    (path to seq-sat-mercury code)'
    echo '4 argument = timeout'
    echo '5 argument = type of search, select between single search (false) and parametrized search (true)'
    echo '6 argument (not required if arg 5 is set to true) = cost type (suggested 1 or 2)'
    echo '7 argument (not required if arg 5 is set to true) = weigth (suggested 1-5)'
    printf '\nEXAMPLE (single search) :\n'
    echo './mercury'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/domain.pddl'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/problems/p1.pddl'
    echo '/home/user/indigo/src/mas_third_party_software/mercury_planner/build/Mercury-fixed/seq-sat-mercury'
    echo '10.0'
    echo 'false'
    echo '2'
    echo '3'
    printf '\nEXAMPLE (parametrized search) :\n'
    echo './mercury'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/domain.pddl'
    echo '/home/user/indigo/src/mas_common_robotics/mcr_task_planning/mcr_knowledge/common/pddl/example_domain/problems/p1.pddl'
    echo '/home/user/indigo/src/mas_third_party_software/mercury_planner/build/Mercury-fixed/seq-sat-mercury'
    echo '10.0'
    echo 'true'
    printf '\nWARNING : Script execution will be aborted\n'
    exit 1
}


#-----------INITIAL CHECKS--------
# --help shows script usage and exits
if [ "${1}" == '--h' ] || [ "${1}" == '--help' ] ; then
    log_mercury "A request from user was received to show help options"
    show_help_and_exit
fi

# check if number of received arguments is ok, if not then prints USAGE instructions and exit
if [[ $# < 5 ]]; then
    log_mercury "Error - Less than 5 arguments were received"
    show_help_and_exit
fi

if [[ $# > 7 ]]; then
    log_mercury "Error - More than 7 arguments were received"
    show_help_and_exit
fi

if [ ${5} != 'true' ] && [ ${5} != 'false' ] ; then
    log_mercury "Error - Argument 5 must be either true or false, received :" ${5}
    show_help_and_exit
fi


#--------------SETUP--------------
# Paths to planner components

# contains the PDDL domain definition
PDDL_DOMAIN_PATH=${1}

# contains PDDL problem instance
PDDL_PROBLEM_PATH=${2}

# the directoy which holds mercury_planner code
BASEDIR=${3}'/src'

# timeout value
SEARCH_TIMEOUT=${4}

# choose search parameters, options (customized, all)
# customized : perform the search only with 1 set of parameters
# all : perform 10 times the search with different sets of predefined parameters
IS_PARAMETRIZED=${5}

# if there are 7 arguments this means that we need to perform 1 search only
# then we need to set cost type and weight parameters
if [[ $# == 7 ]]; then
    SEARCH_COST_TYPE=${6}
    SEARCH_WEIGHT=${7}
fi

# full path to translate.py file
TRANSLATE_PATH="$BASEDIR/translate/translate.py"

# full path to preprocess binary
PREPROCESS_PATH="$BASEDIR/preprocess/preprocess"

# full path to search binary
SEARCH_PATH="$BASEDIR/search/downward-1"

# command used to determine how much time took to plan
TIME="time --output=elapsed.time --format=%S\n%U\n"

# command to run the python file
PYTHON_COMMAND="python"


#--------------CLEAN--------------
function clean {
    #remove output file if exists
    file=output
    if [ ! -e "$file" ]; then
        log_mercury "nothing to clean for : output"
    else 
        log_mercury "clean : removing file output"
        rm output
    fi

    #remove output file if exists
    file=output.sas
    if [ ! -e "$file" ]; then
        log_mercury "nothing to clean for : output.sas"
    else 
        log_mercury "clean : removing file output.sas"
        rm output.sas
    fi

    #remove output file if exists
    file=plan_numbers_and_cost
    if [ ! -e "$file" ]; then
        log_mercury "nothing to clean for : plan_numbers_and_cost"
    else 
        log_mercury "clean : removing file plan_numbers_and_cost"
        rm plan_numbers_and_cost
    fi
}


#-------------FILE EXISTANCE CHECKS--------------
function check_file_existance(){
    if [ ! -f $1 ]; then
        log_mercury "File not found! :" $1
        exit 1
    fi
}

# check for domain.pddl existance, exit with error if not
check_file_existance $PDDL_DOMAIN_PATH

# checking for problem.pddl existance, exit with error if not
check_file_existance $PDDL_PROBLEM_PATH


#--------------RUN----------------
log_mercury "1. Running translator"
$TIME $PYTHON_COMMAND $TRANSLATE_PATH $PDDL_DOMAIN_PATH $PDDL_PROBLEM_PATH

# check if output.sas was produced by translator, exit with error if not
check_file_existance output.sas

log_mercury "2. Running preprocessor"
$TIME --append "$PREPROCESS_PATH" < output.sas

# check if output was produced by preprocessor, exit with error if not
check_file_existance output

log_mercury "3. Running search"

function analize_search_timeout(){
# get and process timeout return value
RETVAL=$?
if [[ $RETVAL == 124 ]]; then
    log_mercury "Timeout, could not complete search in the requested time :" $SEARCH_TIMEOUT "secs"
    
    # exit only if cutomized search is active, otherwise keep searching
    if [ $IS_PARAMETRIZED == "false" ]; then
        clean
        exit 1
    fi
else
    log_mercury "Search function returned without timeout"
fi
}

function search(){
log_mercury "************************SEARCHING**********************"
timeout "$SEARCH_TIMEOUT"s $TIME --append "$SEARCH_PATH" \
--heuristic "hrb=RB(cost_type=1, extract_plan=true, next_red_action_test=true, applicable_paths_first=true, use_connected=true)" \
--heuristic "hlm=lmcount(lm_rhw(reasonable_orders=true,lm_cost_type=$1,cost_type=$1))" \
--search "lazy_wastar([hrb,hlm],preferred=[hrb,hlm],w=$2)" \
--plan-file $3 < output
}

function run_parametrized_search(){
log_mercury "Running parametrized search with 10 different set of parameters"
declare -i parameter_set=1
for i in `seq 1 2`;
do
    for j in `seq 1 5`;
    do
        log_mercury "parameter set :" $parameter_set
        search $i $j mercury.plan.$parameter_set
        analize_search_timeout
        parameter_set=$[parameter_set+1]
    done
done    
}

if [ $IS_PARAMETRIZED == "true" ]; then
    log_mercury "Running multiple search"
    run_parametrized_search
else #TODO print line not working
    log_mercury "Running single search with the following parameters (cost type, search weight) :" "$SEARCH_COST_TYPE" "," "$SEARCH_WEIGHT"
    search $SEARCH_COST_TYPE $SEARCH_WEIGHT mercury.plan
    analize_search_timeout
fi

#remove temp files
clean
