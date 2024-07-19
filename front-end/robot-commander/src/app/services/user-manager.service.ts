import { Injectable } from '@angular/core';
import { BehaviorSubject } from 'rxjs';
import { User } from '../interfaces/User';

@Injectable({
  providedIn: 'root',
})
export class UserManagerService {
  private userSource = new BehaviorSubject<User|null>(null);
  user$ = this.userSource.asObservable();

  setUser(user: User) {
    this.userSource.next(user);
  }

  clearUser() {
    this.userSource.next(null);
  }
}
