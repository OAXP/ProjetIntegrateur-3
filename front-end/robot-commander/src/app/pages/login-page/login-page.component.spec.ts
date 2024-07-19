import { ComponentFixture, TestBed } from '@angular/core/testing';
import { LoginPageComponent } from './login-page.component';
import { Router } from '@angular/router';
import { SocketClientService } from 'src/app/services/socket-client/socket-client.service';
import { FormsModule } from '@angular/forms';
import { MatCardModule } from '@angular/material/card';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { NoopAnimationsModule } from '@angular/platform-browser/animations';
import { of, throwError } from 'rxjs';

class MockSocketClientService {
  OnLoginSuccess = of(null);
  OnLoginError = throwError(() => new Error('Test Error'));
  connectWithUsername = jasmine.createSpy().and.returnValue(Promise.resolve());
}

class MockRouter {
  navigate = jasmine.createSpy('navigate');
}

describe('LoginPageComponent', () => {
  let component: LoginPageComponent;
  let fixture: ComponentFixture<LoginPageComponent>;
  let mockSocketClientService: MockSocketClientService;
  let mockRouter: MockRouter;

  beforeEach(async () => {
    mockSocketClientService = new MockSocketClientService();
    mockRouter = new MockRouter();

    await TestBed.configureTestingModule({
      declarations: [ LoginPageComponent ],
      providers: [
        { provide: SocketClientService, useValue: mockSocketClientService },
        { provide: Router, useValue: mockRouter }
      ],
      imports: [
        FormsModule, 
        MatCardModule, 
        MatFormFieldModule, 
        MatInputModule, 
        BrowserAnimationsModule,
        NoopAnimationsModule
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(LoginPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should navigate to "/status" on successful login', async () => {
    component.username = 'testUser';
    await component.login();
    fixture.detectChanges();
    expect(mockRouter.navigate).toHaveBeenCalledWith(['/status']);
  });

  it('should display error message on login failure', async () => {
    component.username = '';
    await component.login();
    fixture.detectChanges();
    expect(component.errorMessage).toBe('Please enter a username.');
  });
});
