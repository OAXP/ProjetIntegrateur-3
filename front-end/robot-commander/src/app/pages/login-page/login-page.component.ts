import { Component, OnInit, OnDestroy } from '@angular/core';
import { Router } from '@angular/router';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { Subscription } from 'rxjs';

@Component({
  selector: 'app-login-page',
  templateUrl: './login-page.component.html',
  styleUrls: ['./login-page.component.css']
})
export class LoginPageComponent implements OnInit, OnDestroy {
  username: string = '';
  private loginSuccessSub!: Subscription;
  private loginErrorSub!: Subscription;
  errorMessage: string = '';

  constructor(
    private socketService: SocketClientService,
    private router: Router
  ) {}

  ngOnInit(): void {
    this.loginSuccessSub = this.socketService.OnLoginSuccess.subscribe(() => {
      this.router.navigate(['/status']);
    });

    this.loginErrorSub = this.socketService.OnLoginError.subscribe((error: string) => {
      this.errorMessage = error;
      alert(error);
    });
  }

  ngOnDestroy(): void {
    this.loginSuccessSub.unsubscribe();
    this.loginErrorSub.unsubscribe();
  }

  login(): void {
    if (this.username) {
      this.socketService.connectWithUsername(this.username)
      .catch((error) => {
        console.error('Connection error:', error);
      });
    } else {
      this.errorMessage = "Please enter a username.";
    }
  }
}
