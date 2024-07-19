import { UserManagerService } from './user-manager.service';
import { User } from '../interfaces/User';

describe('UserManagerService', () => {
  let service: UserManagerService;
  let setTest: boolean = false;
  let clearTest: boolean = false;

  beforeEach(() => {
    service = new UserManagerService();
  });

  it('should allow setting and emitting a user', (done) => {
    service.user$.subscribe((user: User|null) => {
      if (setTest) {
        expect(user).toEqual({ username: 'testUser', isAdmin: false });
        setTest = false;
        done();
      }
    });
    setTest = true;
    service.setUser({ username: 'testUser', isAdmin: false });
  });  

  it('should allow clearing the user and emitting null', (done) => {
    service.setUser({ username: 'testUser', isAdmin: false });
    service.user$.subscribe((user: User|null) => {
      if (clearTest) {
        expect(user).toBeNull();
        clearTest = false;
        done();
      }
    });
    clearTest = true;
    service.clearUser();
  });
});

