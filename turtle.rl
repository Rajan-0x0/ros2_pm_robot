type Float
type Pose

skillset turtle {

    data {
        pose: Pose period 1.0
    }

    resource {
        Authority {
            state { Teleop Skill }
            initial Teleop
            transition all
        }
        Move {
            state { Moving NotMoving }
            initial NotMoving
            transition all
        }
        Rotate {
            state { Rotating NotRotating }
            initial NotRotating
            transition all
        }
    }

    event {
        authority_to_skill {
            guard Authority == Teleop
            effect Authority -> Skill
        }
        authority_to_teleop {
            effect Authority -> Teleop
        }
    }

    skill MoveForward {
        input {
            distance: Float
            speed: Float
        }
        precondition {
            has_authority: Authority == Skill
            not_moving: Move == NotMoving
        }
        start Move -> Moving
        invariant has_authority {
            guard Authority == Skill
            effect Move -> NotMoving
        }
        interrupt {
            interrupting false
            effect Move -> NotMoving
        }
        success completed {
            effect Move -> NotMoving
        }
    }

    skill RotateAngle {
        input {
            angle: Float
            speed: Float
        }
        precondition {
            has_authority: Authority == Skill
            not_rotating: Rotate == NotRotating
        }
        start Rotate -> Rotating
        invariant has_authority {
            guard Authority == Skill
            effect Rotate -> NotRotating
        }
        interrupt {
            interrupting false
            effect Rotate -> NotRotating
        }
        success completed {
            effect Rotate -> NotRotating
        }
    }
    
    skill MoveInCircle {
        input {
            radius: Float
            speed: Float
        }
        precondition {
            has_authority: Authority == Skill
            not_moving: Move == NotMoving
        }
        start Move -> Moving
        invariant has_authority {
            guard Authority == Skill
            effect Move -> NotMoving
        }
        interrupt {
            interrupting false
            effect Move -> NotMoving
        }
        success completed {
            effect Move -> NotMoving
        }
    }    

}  
      
