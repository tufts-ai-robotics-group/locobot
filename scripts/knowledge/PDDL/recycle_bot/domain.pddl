(define (domain recycle_bot)

;remove requirements that are not needed
(:requirements :strips :typing :negative-preconditions)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    room ball can nothing robot doorway bin - object
)

; un-comment following line if constants are needed
;(:constants )
(:predicates
    (at ?r - room ?o - object)
    (connect ?r1 - room ?r2 - room ?d - doorway)
    (facing ?v0 - object)
    (hold ?o - object)
    (contain ?o1 - object ?o2 - object)
)

(:action approach
    :parameters (?object01 - object ?room01 - room ?x - object) ; approach to object 1 which is in room01 and robot_1 is currently facing x
    :precondition (and
        (at ?room01 ?object01) ; to move to object02
        (at ?room01 robot_1) ; robot
        (facing ?x)
     )
    :effect ( and
    (facing ?object01)
    (not (facing ?x))
    )
)

(:action pass_through_door 
    :parameters (?room01 - room ?room02 - room ?doorway01 - doorway) ; pass through door which connects room01 and room02
    :precondition 
    (and
        (at ?room01 robot_1)
        (connect ?room01 ?room02 ?doorway01)
        (facing ?doorway01)
     )
    :effect 
    (and
        (at ?room02 robot_1)
        (facing nothing)
    )
)

(:action pick
    :parameters (?object01 - object ?room01 - room) ; pick object01 which is in room01
    :precondition (and
        (at ?room01 ?object01)
        (at ?room01 robot_1)
        (facing ?object01)
        (hold nothing)
     )
    :effect (and
        (facing nothing)
        (hold ?object01)
     )
)
(:action place
    :parameters (?object01 - object ?room01 - room ?object02 - object);place object01 inside object02 which is in room room01
    :precondition (and 
        (at ?room01 robot_1)
        (at ?room01 ?object02)
        (facing ?object02)
        (hold ?object01)
    )
    :effect (and 
        (hold nothing)
        (contain ?object01 ?object02)
    )
    )
)