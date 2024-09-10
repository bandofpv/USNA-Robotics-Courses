class User():
    def __init__(self, first_name, last_name, age, race, gender, login_attempts):
        self.first_name = first_name.title()
        self.last_name = last_name.title()
        self.age = age
        self.race = race.title()
        self.gender = gender.title()
        self.login_attempts = login_attempts
    
    def describe_user(self):
        print(f"{self.first_name} {self.last_name} \nage: {self.age} \nrace: {self.race} \ngender: {self.gender}")
        
    def greet_user(self):
        print(f"Hello {self.first_name}!")
    
    def increment_login_attempts(self):
        self.login_attempts += 1
        
    def reset_login_attempts(self):
        self.login_attempts = 0