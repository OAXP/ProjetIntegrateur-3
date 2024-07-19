import { HttpException } from '@src/classes/http.exception';
import * as cookieParser from 'cookie-parser';
import * as cors from 'cors';
import * as express from 'express';
import { StatusCodes } from 'http-status-codes';
import * as swaggerJSDoc from 'swagger-jsdoc';
import * as swaggerUi from 'swagger-ui-express';
import { Service } from 'typedi';
import * as process from 'process';
import * as morgan from 'morgan';
import { CommandController } from './controllers/command.controller';
import { InfoController } from './controllers/info.controller';

@Service()
export class Application {
    app: express.Application;
    private readonly internalError: number = StatusCodes.INTERNAL_SERVER_ERROR;
    private readonly swaggerOptions: swaggerJSDoc.Options;

    constructor(private readonly commandController: CommandController, private readonly infoController: InfoController) {
        this.app = express();

        this.swaggerOptions = {
            swaggerDefinition: {
                openapi: '3.0.0',
                info: {
                    title: 'Cadriciel Serveur',
                    version: '1.0.0',
                },
            },
            apis: ['./src/controllers/*.ts'],
        };

        this.config();

        this.bindRoutes();
    }

    bindRoutes(): void {
        if (this.app.get('env') === 'development') {
            this.app.use(
                '/api/docs',
                swaggerUi.serve,
                swaggerUi.setup(swaggerJSDoc(this.swaggerOptions), {
                    swaggerOptions: { queryConfigEnabled: true },
                })
            );
        }
        this.app.use('/api/command', this.commandController.router);
        this.app.use('/api/info', this.infoController.router);

        if (this.app.get('env') === 'development') {
            this.app.use('/', (req, res) => {
                res.redirect('/api/docs');
            });
        }
        this.errorHandling();
    }

    private config(): void {
        // Middlewares configuration
        this.app.set('trust proxy', true);
        if (process.env.npm_lifecycle_event !== 'test') {
            this.app.use(
                morgan(
                    '--> ":method :url HTTP/:http-version" [:date] :remote-addr :remote-user',
                    {
                        immediate: true,
                    }
                )
            );
            this.app.use(
                morgan(
                    '<-- ":method :url HTTP/:http-version" [:date] :status :res[content-length]',
                    {
                        immediate: false,
                    }
                )
            );
        }
    
        this.app.use(express.json({ limit: '50mb' }));
        this.app.use(express.urlencoded({ extended: true }));
        this.app.use(cookieParser());
        this.app.use(
            cors({
                methods: ['GET', 'POST', 'PATCH', 'DELETE', 'PUT'],
                origin: "*",
            })
        );
        this.app.options(
            '*',
            cors({
                methods: ['GET', 'POST', 'PATCH', 'DELETE', 'PUT'],
                origin: "*",
            })
        );
    }
    

    private errorHandling(): void {
        // When previous handlers have not served a request: path wasn't found
        this.app.use(
            (
                req: express.Request,
                res: express.Response,
                next: express.NextFunction
            ) => {
                const err: HttpException = new HttpException('Not Found');
                next(err);
            }
        );

        // development error handler
        // will print stacktrace
        if (this.app.get('env') === 'development') {
            this.app.use(
                (err: HttpException, req: express.Request, res: express.Response) => {
                    res.status(err.status || this.internalError);
                    res.send({
                        message: err.message,
                        error: err,
                    });
                }
            );
        }

        // production error handler
        // no stacktraces  leaked to user (in production .env only)
        this.app.use(
            (err: HttpException, req: express.Request, res: express.Response) => {
                res.status(err.status || this.internalError);
                res.send({
                    message: err.message,
                    error: {},
                });
            }
        );
    }
}
