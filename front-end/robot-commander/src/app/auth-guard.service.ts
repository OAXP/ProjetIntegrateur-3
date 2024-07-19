import { Router } from '@angular/router';
import { inject } from '@angular/core';
import { SocketClientService } from './services/socket-client/socket-client.service';

export function authGuard() {
  const router = inject(Router);
  const socketService = inject(SocketClientService);

  if (socketService.isSocketAlive()) {
    return true;
  } else {
    router.navigate(['/login']);
    return false;
  }
}