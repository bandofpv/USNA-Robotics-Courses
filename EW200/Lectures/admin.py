from user_class import User

class Privileges():
    def __init__(self):
        self.privileges = ['can add post', 'can delete post', 'can ban user', 'can create user']
        
    def show_privileges(self):
        print("You have the following privileges: ")
        for privilege in self.privileges:
            print(f"- {privilege}")

class Admin(User):
    def __init__(self, first_name, last_name, age, race, gender, login_attempts):
        super().__init__(first_name, last_name, age, race, gender, login_attempts)
        self.privileges = Privileges()