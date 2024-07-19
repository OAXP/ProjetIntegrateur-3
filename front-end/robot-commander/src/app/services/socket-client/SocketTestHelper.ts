type CallbackSignature = (...params: any[]) => void;

export class SocketTestHelper {
    private callbacks = new Map<string, CallbackSignature[]>();

    on(event: string, callback: CallbackSignature): void {
        if (!this.callbacks.has(event)) {
            this.callbacks.set(event, []);
        }

        this.callbacks.get(event)?.push(callback);
    }

    // eslint-disable-next-line no-unused-vars
    emit(event: string): void {
        return;
    }

    simulateEvent(event: string, ...args: any[]) {
        if (this.callbacks.has(event)) {
            console.log(`Simulating event: ${event} with args`, args);
            this.callbacks.get(event)?.forEach(callback => {
                callback(...args);
            });
        } else {
            console.log(`No callbacks for event: ${event}`);
        }
    }
    

    disconnect(): void {
        return;
    }
}
