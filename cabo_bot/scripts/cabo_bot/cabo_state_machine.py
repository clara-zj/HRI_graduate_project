import pygame


################################################################
# Constants for state names
INITING = 'initing'
STANDBY = 'standby'
# short moves
BINDING = 'binding'
FORWARDING = 'forwarding'
BACKWARDING = 'backwarding'
LEFTING = 'lefting'
RIGHTING = 'righting'
# long moves
DANCING = 'dancing'
LONGFORWARDING = 'longforwarding'
FOLLOWING = 'following'


################################################################
class State:
    def __init__(self, controller, name):
        self.controller = controller
        self.name = name

    def getStateName(self):
        return self.name

    def enter(self):
        print(f"Entering {self.name} State")
        return False

    def handle_started(self):
        print(f"Invalid [handle_started] operation in {self.name} State")
        return False

    def handle_follow(self):
        print(f"Invalid [handle_follow] operation in {self.name} State")
        return False

    def handle_forward(self):
        print(f"Invalid [handle_forward] operation in {self.name} State")
        return False
    
    def handle_left(self):
        print(f"Invalid [handle_left] operation in {self.name} State")
        return False

    def handle_right(self):
        print(f"Invalid [handle_right] operation in {self.name} State")
        return False

    def handle_backward(self):
        print(f"Invalid [handle_backward] operation in {self.name} State")
        return False

    def handle_dance(self):
        print(f"Invalid [handle_dance] operation in {self.name} State")
        return False

    def handle_long_forward(self):
        print(f"Invalid [handle_long_forward] operation in {self.name} State")
        return False

    def handle_short_finished(self):
        print(f"Invalid [handle_short_finished] operation in {self.name} State")
        return False

    def handle_stop(self):
        print(f"Invalid [handle_stop] operation in {self.name} State")
        return False

    def handle_bind(self):
        print(f"Invalid [handle_bind] operation in {self.name} State")
        return False

    def handle_binded(self):
        print(f"Invalid [handle_binded] operation in {self.name} State")
        return False

    def handle_bind_faild(self):
        print(f"Invalid [handle_bind_faild] operation in {self.name} State")
        return False
    
    def handle_sit(self):
        print(f"Invalid [handle_sit] operation in {self.name} State")
        return False

################################################################
class InitingState(State):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def handle_started(self):
        self.controller.set_state(STANDBY)
        return True


################################################################
class StandByState(State):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_standby()
    
    def handle_sit(self):
        self.controller.set_state(INITING)
        if self.controller.node is not None:
            self.controller.node.move_sit()
        return True

    # short move
    def handle_forward(self):
        self.controller.set_state(FORWARDING)
        return True

    def handle_backward(self):
        self.controller.set_state(BACKWARDING) 
        return True

    def handle_left(self):
        self.controller.set_state(LEFTING)
        return True

    def handle_right(self):
        self.controller.set_state(RIGHTING)
        return True

    def handle_bind(self):
        self.controller.set_state(BINDING)   
        return True

    def handle_dance(self):
        self.controller.set_state(DANCING)
        return True

    # long move
    def handle_long_forward(self):
        self.controller.set_state(LONGFORWARDING)
        return True
 
    def handle_follow(self):
        self.controller.set_state(FOLLOWING)     
        return True


################################################################
class ShortMoveState(State):
    def __init__(self, controller, name):
        super().__init__(controller, name)
   
    def handle_short_finished(self):
        self.controller.set_state(STANDBY)
        return True


#----------------------------------------------------------------
class LongMoveState(State):
    def __init__(self, controller, name):
        super().__init__(controller, name)        

    def handle_stop(self):
        self.controller.set_state(STANDBY)
        return True

################################################################
class BindingState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_bind()

    def handle_short_finished(self):
        return False

    def handle_binded(self):
        self.controller.is_binded = True
        self.controller.set_state(STANDBY)
        return True

    def handle_bind_faild(self):
        self.controller.is_binded = False
        self.controller.set_state(STANDBY)
        return True


#----------------------------------------------------------------
class ForwardState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)
  
    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_forward()


#----------------------------------------------------------------
class BackwardState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)
    
    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_backward()


#----------------------------------------------------------------
class DanceState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_dance()


#----------------------------------------------------------------
class LeftingState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_left()


#----------------------------------------------------------------
class RightingState(ShortMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_right()


################################################################
class FollowState(LongMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)

    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_follow()

    def handle_stop(self):
        if self.controller.node is not None:
            self.controller.node.move_stop_vision()
        self.controller.set_state(STANDBY)
        return True


#----------------------------------------------------------------
class LongForwardState(LongMoveState):
    def __init__(self, controller, name):
        super().__init__(controller, name)
        
    def enter(self):
        super().enter()
        if self.controller.node is not None:
            self.controller.node.move_long_forward()

################################################################
class StateController:
    def __init__(self, node=None):
        self.states = {
            INITING: InitingState(self, INITING),
            STANDBY: StandByState(self, STANDBY),
            BINDING: BindingState(self, BINDING),
            DANCING: DanceState(self, DANCING),
            FOLLOWING: FollowState(self, FOLLOWING),
            BACKWARDING: BackwardState(self, BACKWARDING),
            FORWARDING: ForwardState(self, FORWARDING),
            LEFTING: LeftingState(self, LEFTING),
            RIGHTING: RightingState(self, RIGHTING),
            LONGFORWARDING: LongForwardState(self, LONGFORWARDING)
        }
        self.current_state = self.states[INITING]
        self.current_state.enter()
        self.is_binded = False
        self.node = node

    def set_state(self, state_name):
        if state_name in self.states:
            self.current_state = self.states[state_name]
            self.current_state.enter()

    def getStateName(self):
        return self.current_state.getStateName()

    def handle_started(self):
        return self.current_state.handle_started()

    def handle_follow(self):
        if self.is_binded:
            return self.current_state.handle_follow()
        else:
            return False

    def handle_sit(self):
        return self.current_state.handle_sit()
    
    def handle_forward(self):
        return self.current_state.handle_forward()

    def handle_backward(self):
        return self.current_state.handle_backward()

    def handle_left(self):
        return self.current_state.handle_left()

    def handle_right(self):
        return self.current_state.handle_right()

    def handle_dance(self):
        return self.current_state.handle_dance()

    def handle_long_forward(self):
        return self.current_state.handle_long_forward()

    def handle_short_finished(self):
        return self.current_state.handle_short_finished()

    def handle_stop(self):
        return self.current_state.handle_stop()

    def handle_bind(self):
        return self.current_state.handle_bind()

    def handle_binded(self):
        return self.current_state.handle_binded()

    def handle_bind_faild(self):
        return self.current_state.handle_bind_faild()
    
    def is_following(self):
        return self.current_state.getStateName() == FOLLOWING


################################################################
def state_unit_test(real_state, desired_state):
    assert real_state == desired_state, f"Desired state: {desired_state}, Real state: {real_state}"


if __name__ == "__main__":
    # init
    controller = StateController()
    state_unit_test(controller.getStateName(), INITING)

    controller.handle_started()
    state_unit_test(controller.getStateName(), STANDBY)

    # forward
    state_unit_test(controller.handle_forward(), True)
    state_unit_test(controller.getStateName(), FORWARDING)

    state_unit_test(controller.handle_short_finished(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # backward
    state_unit_test(controller.handle_backward(), True)
    state_unit_test(controller.getStateName(), BACKWARDING)

    state_unit_test(controller.handle_short_finished(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # left
    state_unit_test(controller.handle_left(), True)
    state_unit_test(controller.getStateName(), LEFTING)

    state_unit_test(controller.handle_short_finished(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # right
    state_unit_test(controller.handle_right(), True)
    state_unit_test(controller.getStateName(), RIGHTING)

    state_unit_test(controller.handle_short_finished(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # bind faild
    state_unit_test(controller.is_binded, False)
    state_unit_test(controller.handle_bind(), True)
    state_unit_test(controller.getStateName(), BINDING)

    state_unit_test(controller.handle_bind_faild(), True)
    state_unit_test(controller.getStateName(), STANDBY)
    state_unit_test(controller.is_binded, False)

    # follow faild
    state_unit_test(controller.handle_follow(), False)  # cannot follow when not binded
    state_unit_test(controller.getStateName(), STANDBY)

    # bind success
    state_unit_test(controller.is_binded, False)
    state_unit_test(controller.handle_bind(), True)
    state_unit_test(controller.getStateName(), BINDING)

    state_unit_test(controller.handle_short_finished(), False)  # should not change state
    state_unit_test(controller.getStateName(), BINDING)

    state_unit_test(controller.handle_binded(), True)
    state_unit_test(controller.getStateName(), STANDBY)
    state_unit_test(controller.is_binded, True)

    # follow
    state_unit_test(controller.handle_follow(), True)
    state_unit_test(controller.getStateName(), FOLLOWING)

    state_unit_test(controller.handle_stop(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # long forward
    state_unit_test(controller.handle_long_forward(), True)
    state_unit_test(controller.getStateName(), LONGFORWARDING)
    state_unit_test(controller.handle_stop(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # dance
    state_unit_test(controller.handle_dance(), True)
    state_unit_test(controller.getStateName(), DANCING)

    state_unit_test(controller.handle_short_finished(), True)
    state_unit_test(controller.getStateName(), STANDBY)

    # long move - short move - standby
    # controller.handle_follow()
    # state_unit_test(controller.getStateName(), FOLLOWING)

   

    # # dance
    # controller.handle_dance()
    # state_unit_test(controller.getStateName(), DANCING)

    # controller.handle_finished()
    # state_unit_test(controller.getStateName(), STANDBY)

