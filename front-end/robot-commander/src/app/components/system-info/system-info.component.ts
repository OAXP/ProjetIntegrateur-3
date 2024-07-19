import { Component, OnInit } from '@angular/core';
import { UserManagerService } from 'src/app/services/user-manager.service';
import { User } from 'src/app/interfaces/User';

@Component({
  selector: 'app-system-info',
  templateUrl: './system-info.component.html',
  styleUrls: ['./system-info.component.css']
})
export class SystemInfoComponent implements OnInit {
  user: User|null = null;

  constructor(private userService: UserManagerService) { }

  ngOnInit() {
    this.userService.user$.subscribe(user => {
      this.user = user;
    });
  }
}
