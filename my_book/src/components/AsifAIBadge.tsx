import React from 'react';
import clsx from 'clsx';
import styles from './AsifAIBadge.module.css';

type AsifAIBadgeItem = {
  title: string;
  description: JSX.Element;
};

const AsifAIBadgeList: AsifAIBadgeItem[] = [
  {
    title: 'AI-Powered',
    description: <>Built with modern AI technologies and best practices</>,
  },
  {
    title: 'Comprehensive',
    description: <>Complete coverage from fundamentals to advanced topics</>,
  },
  {
    title: 'Practical',
    description: <>Real-world examples and hands-on implementation guides</>,
  },
];

function AsifAIBadgeItem({title, description}: AsifAIBadgeItem): JSX.Element {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function AsifAIBadge(): JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {AsifAIBadgeList.map((props, idx) => (
            <AsifAIBadgeItem key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}