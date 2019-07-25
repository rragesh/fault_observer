(define (domain SCAN)
    (:requirements
        :strips
        :typing
    )
    (:predicates    (stable ?P)
                    (continue)

    )
    (:action process_PCL
        :parameters (?P0 ?P1 ?P2 ?P3)
        :precondition (and (stable ?P0) (stable ?P1) (stable ?P2) (stable ?P3))
        :effect (continue)
    )
)
